"""Bounded keyvector solver for the DG-5F hand."""

from __future__ import annotations

from dataclasses import dataclass
import math
import time

import numpy as np
from scipy.optimize import least_squares

from .calibration import default_calibration, transform_hand
from .dg5f_model import DEFAULT_REFERENCE_JOINTS_RAD, DG5FHandModel
from .keyvector_loss import (
    DEFAULT_CONTACT_WEIGHT_S1,
    DEFAULT_CONTACT_WEIGHT_S2,
    KEYVECTOR_NAMES,
    build_target_vectors,
    compute_keyvectors,
)
from .manus_landmarks import ManusHandLandmarks
from .manus_joints import default_joint_prior_weights


@dataclass(frozen=True)
class SolveResult:
    """One keyvector-solver result."""

    joint_positions: np.ndarray
    success: bool
    cost: float
    solve_time_ms: float
    message: str
    human_hand_mapped: ManusHandLandmarks
    human_keypoints: dict[str, dict[str, np.ndarray]]
    human_vectors: dict[str, np.ndarray]
    robot_keypoints: dict[str, dict[str, np.ndarray]]
    robot_vectors: dict[str, np.ndarray]
    target_vectors: dict[str, np.ndarray]
    weights: dict[str, float]
    q_seed: np.ndarray
    q_prior: np.ndarray | None
    prior_weights: np.ndarray | None


class KeyvectorRetargetSolver:
    """Solve DG-5F joints from mapped human keyvectors."""

    def __init__(
        self,
        model: DG5FHandModel,
        calibration: dict | None = None,
        reference_joint_positions: np.ndarray | None = None,
        epsilon_contact_m: float = 0.015,
        eta_thumb_close_m: float = 0.008,
        eta_interfinger_sep_m: float = 0.018,
        lambda_smooth: float = 0.0,
        solver_max_nfev: int = 20,
        solver_tol: float = 1e-4,
        contact_weight_s1: float = DEFAULT_CONTACT_WEIGHT_S1,
        contact_weight_s2: float = DEFAULT_CONTACT_WEIGHT_S2,
        prior_seed_alpha: float = 0.7,
        joint_prior_weight_flex: float = 0.05,
        joint_prior_weight_spread: float = 0.03,
    ):
        self.model = model
        self.calibration = default_calibration() if calibration is None else dict(default_calibration(), **calibration)
        self.reference_joint_positions = (
            np.copy(DEFAULT_REFERENCE_JOINTS_RAD) if reference_joint_positions is None else model.clip_to_limits(reference_joint_positions)
        )
        self.epsilon_contact_m = float(epsilon_contact_m)
        self.eta_thumb_close_m = float(eta_thumb_close_m)
        self.eta_interfinger_sep_m = float(eta_interfinger_sep_m)
        self.lambda_smooth = float(lambda_smooth)
        self.solver_max_nfev = int(solver_max_nfev)
        self.solver_tol = float(solver_tol)
        self.contact_weight_s1 = float(contact_weight_s1)
        self.contact_weight_s2 = float(contact_weight_s2)
        self.prior_seed_alpha = float(np.clip(prior_seed_alpha, 0.0, 1.0))
        self.joint_prior_weight_flex = float(joint_prior_weight_flex)
        self.joint_prior_weight_spread = float(joint_prior_weight_spread)
        self.default_prior_weights = default_joint_prior_weights(
            self.model.joint_names,
            flex_weight=self.joint_prior_weight_flex,
            spread_weight=self.joint_prior_weight_spread,
        )
        self.q_prev = np.copy(self.reference_joint_positions)
        self.betas = {
            name: float(beta)
            for name, beta in zip(KEYVECTOR_NAMES, self.calibration.get('betas', [1.0] * len(KEYVECTOR_NAMES)))
        }

    def reset(self, joint_positions: np.ndarray | None = None) -> None:
        self.q_prev = np.copy(self.reference_joint_positions if joint_positions is None else self.model.clip_to_limits(joint_positions))

    def ease_toward_reference(self, alpha: float) -> np.ndarray:
        alpha = float(np.clip(alpha, 0.0, 1.0))
        self.q_prev = self.model.clip_to_limits((1.0 - alpha) * self.q_prev + alpha * self.reference_joint_positions)
        return np.copy(self.q_prev)

    def _seed_from_prior(self, q_prior: np.ndarray | None) -> np.ndarray:
        if q_prior is None:
            return np.copy(self.q_prev)
        q_prior = self.model.clip_to_limits(q_prior)
        return self.model.clip_to_limits(self.prior_seed_alpha * q_prior + (1.0 - self.prior_seed_alpha) * self.q_prev)

    def _residual(
        self,
        q: np.ndarray,
        target_vectors: dict[str, np.ndarray],
        weights: dict[str, float],
        q_anchor: np.ndarray,
        q_prior: np.ndarray | None,
        prior_weights: np.ndarray | None,
    ) -> np.ndarray:
        robot_keypoints = self.model.get_virtual_keypoints(q)
        robot_vectors = compute_keyvectors(robot_keypoints)

        residuals = []
        for name in KEYVECTOR_NAMES:
            residuals.extend((math.sqrt(weights[name]) * (robot_vectors[name] - target_vectors[name])).tolist())
        if q_prior is not None and prior_weights is not None:
            residuals.extend((np.sqrt(prior_weights) * (q - q_prior)).tolist())
        if self.lambda_smooth > 0.0:
            residuals.extend((math.sqrt(self.lambda_smooth) * (q - q_anchor)).tolist())
        return np.asarray(residuals, dtype=float)

    def solve(
        self,
        raw_hand: ManusHandLandmarks,
        q_prior: np.ndarray | None = None,
        prior_weights: np.ndarray | None = None,
    ) -> SolveResult:
        human_hand_mapped = transform_hand(raw_hand, self.calibration)
        human_keypoints = human_hand_mapped.loss_keypoints()
        human_vectors = compute_keyvectors(human_keypoints)
        target_bundle = build_target_vectors(
            human_vectors=human_vectors,
            betas=self.betas,
            epsilon_contact_m=self.epsilon_contact_m,
            eta_thumb_close_m=self.eta_thumb_close_m,
            eta_interfinger_sep_m=self.eta_interfinger_sep_m,
            contact_weight_s1=self.contact_weight_s1,
            contact_weight_s2=self.contact_weight_s2,
        )

        q_anchor = np.copy(self.q_prev)
        q_prior_used = None if q_prior is None else self.model.clip_to_limits(np.asarray(q_prior, dtype=float))
        prior_weights_used = None
        if q_prior_used is not None:
            prior_weights_used = self.default_prior_weights if prior_weights is None else np.asarray(prior_weights, dtype=float)
        q_seed = self._seed_from_prior(q_prior_used)
        t0 = time.perf_counter()
        try:
            result = least_squares(
                fun=lambda q: self._residual(
                    q,
                    target_bundle.vectors,
                    target_bundle.weights,
                    q_anchor,
                    q_prior_used,
                    prior_weights_used,
                ),
                x0=q_seed,
                bounds=(self.model.lower_limits, self.model.upper_limits),
                method='trf',
                xtol=self.solver_tol,
                ftol=self.solver_tol,
                gtol=self.solver_tol,
                max_nfev=self.solver_max_nfev,
            )
            solve_time_ms = 1000.0 * (time.perf_counter() - t0)
            success = bool(result.success and np.isfinite(result.x).all())
            if success:
                q_sol = self.model.clip_to_limits(result.x)
                self.q_prev = np.copy(q_sol)
                cost = float(result.cost)
                message = result.message
            else:
                q_sol = np.copy(q_anchor)
                cost = float(result.cost) if hasattr(result, 'cost') else float('inf')
                message = result.message
        except Exception as exc:
            solve_time_ms = 1000.0 * (time.perf_counter() - t0)
            success = False
            q_sol = np.copy(q_anchor)
            cost = float('inf')
            message = f'least_squares exception: {exc}'

        robot_keypoints = self.model.get_virtual_keypoints(q_sol)
        robot_vectors = compute_keyvectors(robot_keypoints)
        return SolveResult(
            joint_positions=q_sol,
            success=success,
            cost=cost,
            solve_time_ms=solve_time_ms,
            message=message,
            human_hand_mapped=human_hand_mapped,
            human_keypoints=human_keypoints,
            human_vectors=human_vectors,
            robot_keypoints=robot_keypoints,
            robot_vectors=robot_vectors,
            target_vectors=target_bundle.vectors,
            weights=target_bundle.weights,
            q_seed=q_seed,
            q_prior=None if q_prior_used is None else np.copy(q_prior_used),
            prior_weights=None if prior_weights_used is None else np.copy(prior_weights_used),
        )
