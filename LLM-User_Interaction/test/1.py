
"""
LLM Learning Dynamics (LLD) — Simulation & Plotting
===================================================
Implements the individual- and population-level model described in our design.
Usage:
    python LLD_sim.py
The script will:
  1) Simulate N individuals over T time-steps
  2) Save figures into ./out/
  3) Print summary statistics
Author: (you)
"""

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from dataclasses import dataclass, asdict
from typing import Dict, Tuple
import pathlib

# -----------------------------
# Utility
# -----------------------------
def clamp(x, lo=0.0, hi=1.0):
    return np.minimum(np.maximum(x, lo), hi)

# -----------------------------
# Model Parameters
# -----------------------------
@dataclass
class GlobalParams:
    T: int = 200                 # time steps
    dt: float = 0.1              # step size
    N: int = 600                 # population size
    seed: int = 7

    # LLM environment over time (can be time-varying; here static constants)
    Q: float = 0.75              # model quality
    H: float = 0.08              # hallucination/error rate
    S: float = 0.6               # scaffolding strength
    P: float = 0.4               # personalization degree
    R: float = 0.5               # retrieval/tools

    # Interaction elasticities / protections
    rho: float = 0.75            # complementarity weight (higher => more "Matthew" tendency)
    sigma: float = 0.8           # scaffolding mitigates complementarity exponent
    theta: float = 0.6           # scaffolding protects against harm
    zeta: float = 0.5            # metacognition protects against harm

    # Lambda coefficients (human side)
    eta1: float = 0.6            # K weight
    eta2: float = 0.2            # B weight
    eta3: float = 0.2            # M weight

    # Lambda coefficients (LLM side)
    kappa1: float = 0.7          # Q
    kappa2: float = 0.3          # R

    # Delta coefficients
    lambda1: float = 0.7         # H contribution
    lambda2: float = 0.3         # W contribution

    # Gamma coefficients
    gamma0: float = 0.1
    gamma1: float = 0.4          # by M
    gamma2: float = 0.4          # by S
    gamma3: float = 0.2          # by P

    # Individual learning dynamics (means and dispersions for heterogeneity)
    alpha_mu: float = 0.30       # natural learning efficiency
    beta_mu: float = 0.70        # LLM-amplified efficiency
    phi_mu: float = 0.03         # forgetting
    psi_mu: float = 0.05         # mislearning sensitivity

    alpha_sd: float = 0.05
    beta_sd: float = 0.10
    phi_sd: float = 0.01
    psi_sd: float = 0.02

    # Metacognition / misuse dynamics
    mu1: float = 0.15            # S*U improves M
    mu2: float = 0.10            # misuse harms M
    mu3: float = 0.02            # regression to mean
    omega1: float = 0.12         # U increases misuse (in unguarded contexts)
    omega2: float = 0.10         # S reduces misuse
    omega3: float = 0.08         # supervision/self-test reduces misuse (we model as part of S)

    # Adoption dynamics
    xi1: float = 0.20            # benefit threshold slope
    xi2: float = 0.10            # social exposure
    xi3: float = 0.08            # frictions

    adopt_threshold: float = 0.02    # minimum perceived net benefit

    # Efficiency (output Y) multiplier parameter
    upsilon: float = 0.8


@dataclass
class ScenarioToggles:
    """Switch between scenarios by overriding some environment knobs."""
    # scenario name
    name: str = "baseline"

    # optional overrides (None => use GlobalParams defaults)
    Q: float = None
    H: float = None
    S: float = None
    P: float = None
    R: float = None
    # Optionally decrease complementarity to model strong scaffolding
    rho: float = None


# -----------------------------
# Initialization
# -----------------------------
def init_population(g: GlobalParams) -> Dict[str, np.ndarray]:
    rng = np.random.default_rng(g.seed)

    # Knowledge capacity (upper bound) — vary by person
    Kbar = rng.uniform(0.7, 1.0, size=g.N)
    # Baseline knowledge, metacognition, misuse
    K0 = rng.beta(2.0, 5.0, size=g.N) * 0.5
    M0 = rng.beta(2.0, 3.0, size=g.N) * 0.6
    B  = rng.beta(3.0, 3.0, size=g.N) * 0.7 + 0.1  # slow variable (prior literacy)
    W0 = rng.beta(2.0, 6.0, size=g.N) * 0.5

    # Time allocation A and starting adoption U
    A  = rng.uniform(0.5, 1.0, size=g.N)
    U0 = rng.beta(1.2, 2.5, size=g.N) * 0.4

    # Heterogeneous structural parameters
    alpha = np.clip(rng.normal(g.alpha_mu, g.alpha_sd, size=g.N), 0.01, 1.0)
    beta  = np.clip(rng.normal(g.beta_mu,  g.beta_sd,  size=g.N), 0.01, 2.0)
    phi   = np.clip(rng.normal(g.phi_mu,   g.phi_sd,   size=g.N), 0.001, 0.2)
    psi   = np.clip(rng.normal(g.psi_mu,   g.psi_sd,   size=g.N), 0.001, 0.3)

    # Frictions and social exposure baseline for adoption
    friction = rng.uniform(0.0, 0.6, size=g.N)
    social0  = rng.uniform(0.0, 0.5, size=g.N)

    return dict(
        K=K0, M=M0, W=W0, B=B, A=A, U=U0, Kbar=Kbar,
        alpha=alpha, beta=beta, phi=phi, psi=psi,
        friction=friction, social=social0
    )

# -----------------------------
# Core Interaction Functions
# -----------------------------
def Lambda_i(K, B, M, g: GlobalParams) -> float:
    """
    Beneficial learning efficiency from human-LLM complementarity.
    """
    human_term = g.eta1 * K + g.eta2 * B + g.eta3 * M
    llm_term   = g.kappa1 * g.Q + g.kappa2 * g.R

    # Scaffolding reduces exponent => reduces complementarity (favors the less-prepared)
    exponent = g.rho * (1.0 - g.sigma * g.S)
    human_term = np.clip(human_term, 1e-6, None)
    llm_term   = np.clip(llm_term,   1e-6, None)

    return (human_term ** exponent) * (llm_term ** (1.0 - exponent))

def Delta_i(W, g: GlobalParams) -> float:
    """
    Mislearning/overreliance loss term.
    """
    harm = g.lambda1 * g.H + g.lambda2 * W
    harm *= (1.0 - g.theta * g.S)  # scaffolding protection
    return clamp(harm, 0.0, 1.0)

def Gamma_i(M, g: GlobalParams) -> float:
    """
    Internalization/transfer probability of LLM content.
    """
    gamma = g.gamma0 + g.gamma1 * M + g.gamma2 * g.S + g.gamma3 * g.P
    return clamp(gamma, 0.0, 1.0)

def perceived_benefit(K, M, g: GlobalParams) -> float:
    """
    A simple proxy for perceived net benefit to drive adoption.
    """
    lam = Lambda_i(K, B=0.6, M=M, g=g)  # use typical B for rough perception
    gam = Gamma_i(M, g)
    ben = g.upsilon * g.Q * lam * gam - g.H * 0.3  # penalize hallucination
    return ben

# -----------------------------
# Dynamics Update
# -----------------------------
def step(pop: Dict[str, np.ndarray], g: GlobalParams) -> Dict[str, np.ndarray]:
    K, M, W, B, A, U, Kbar = pop["K"], pop["M"], pop["W"], pop["B"], pop["A"], pop["U"], pop["Kbar"]
    alpha, beta, phi, psi = pop["alpha"], pop["beta"], pop["phi"], pop["psi"]
    friction, social = pop["friction"], pop["social"]

    # Core interaction terms
    Lam = Lambda_i(K, B, M, g)
    Del = Delta_i(W, g)
    Gam = Gamma_i(M, g)

    # Knowledge change (Eq. 1 discretized)
    dK = (
        alpha * A * (1.0 - K / np.maximum(Kbar, 1e-6)) +
        beta  * A * U * Lam * Gam -
        phi   * K -
        psi   * A * U * Del
    ) * g.dt

    # Update K with bounds [0, Kbar]
    K_new = np.clip(K + dK, 0.0, Kbar)

    # Metacognition dynamics
    dM = (g.mu1 * g.S * U - g.mu2 * W - g.mu3 * (M - 0.5)) * g.dt
    M_new = clamp(M + dM, 0.0, 1.0)

    # Misuse dynamics
    dW = (g.omega1 * U - g.omega2 * g.S - g.omega3 * g.S) * g.dt
    W_new = clamp(W + dW, 0.0, 1.0)

    # Adoption dynamics
    benefit = perceived_benefit(K, M, g)
    social = clamp(social + 0.02 * (np.mean(U) - social), 0.0, 1.0)  # simple diffusion of exposure
    dU = (g.xi1 * np.maximum(benefit - g.adopt_threshold, 0.0) + g.xi2 * social - g.xi3 * friction) * g.dt
    U_new = clamp(U + dU, 0.0, 1.0)

    pop.update(K=K_new, M=M_new, W=W_new, U=U_new, social=social)
    return pop

# -----------------------------
# Output Measure
# -----------------------------
def output_Y(K: np.ndarray, U: np.ndarray, g: GlobalParams) -> np.ndarray:
    """
    Observable productivity/quality measure: Y = g(K) * efficiency(U)
    We use g(K) = K (monotone), efficiency = 1 + upsilon * U * Q
    """
    return K * (1.0 + g.upsilon * U * g.Q)

# Gini coefficient for inequality over K
def gini(x: np.ndarray) -> float:
    x = np.sort(np.clip(x, 0.0, None))
    n = x.size
    if n == 0:
        return 0.0
    cumx = np.cumsum(x)
    denom = n * np.sum(x)
    if denom == 0:
        return 0.0
    return (n + 1 - 2 * np.sum(cumx) / cumx[-1]) / n

# -----------------------------
# Scenario runner
# -----------------------------
def run_scenario(g: GlobalParams, scenario: ScenarioToggles) -> Dict[str, np.ndarray]:
    # Apply overrides
    g_local = GlobalParams(**asdict(g))
    for key in ["Q","H","S","P","R","rho"]:
        val = getattr(scenario, key)
        if val is not None:
            setattr(g_local, key, val)

    pop = init_population(g_local)

    hist_K = np.zeros((g_local.T, g_local.N))
    hist_U = np.zeros((g_local.T, g_local.N))
    hist_M = np.zeros((g_local.T, g_local.N))
    hist_W = np.zeros((g_local.T, g_local.N))
    hist_G = np.zeros(g_local.T)   # gini over K
    hist_Y = np.zeros((g_local.T, g_local.N))

    for t in range(g_local.T):
        hist_K[t] = pop["K"]
        hist_U[t] = pop["U"]
        hist_M[t] = pop["M"]
        hist_W[t] = pop["W"]
        hist_G[t] = gini(pop["K"])
        hist_Y[t] = output_Y(pop["K"], pop["U"], g_local)
        pop = step(pop, g_local)

    results = dict(
        K=hist_K, U=hist_U, M=hist_M, W=hist_W, G=hist_G, Y=hist_Y, params=g_local
    )
    return results

# -----------------------------
# Plotting
# -----------------------------
def ensure_out():
    out_dir = pathlib.Path("out")
    out_dir.mkdir(exist_ok=True, parents=True)
    return out_dir

def plot_time_series(res: Dict[str, np.ndarray], name: str):
    out = ensure_out()
    K = res["K"]; U = res["U"]; M = res["M"]; W = res["W"]; G = res["G"]; Y = res["Y"]
    T = K.shape[0]

    # Means over time
    Kmean = K.mean(axis=1)
    Umean = U.mean(axis=1)
    Mmean = M.mean(axis=1)
    Wmean = W.mean(axis=1)
    Ymean = Y.mean(axis=1)

    # 1) Knowledge mean and 10/90 percentiles
    p10 = np.percentile(K, 10, axis=1)
    p90 = np.percentile(K, 90, axis=1)
    fig1 = plt.figure()
    plt.plot(Kmean, label="Mean K")
    plt.plot(p10, label="P10 K", linestyle="--")
    plt.plot(p90, label="P90 K", linestyle="--")
    plt.title(f"Knowledge Trajectory — {name}")
    plt.xlabel("Time")
    plt.ylabel("K")
    plt.legend()
    fig1.tight_layout()
    fig1.savefig(out / f"{name}_K_time.png")
    plt.close(fig1)

    # 2) Inequality (Gini) over time
    fig2 = plt.figure()
    plt.plot(G, label="Gini(K)")
    plt.title(f"Inequality over time — {name}")
    plt.xlabel("Time")
    plt.ylabel("Gini")
    plt.legend()
    fig2.tight_layout()
    fig2.savefig(out / f"{name}_Gini_time.png")
    plt.close(fig2)

    # 3) Adoption and Metacognition and Misuse
    fig3 = plt.figure()
    plt.plot(Umean, label="Mean U")
    plt.plot(Mmean, label="Mean M")
    plt.plot(Wmean, label="Mean W")
    plt.title(f"Adoption / Metacognition / Misuse — {name}")
    plt.xlabel("Time")
    plt.ylabel("Level")
    plt.legend()
    fig3.tight_layout()
    fig3.savefig(out / f"{name}_U_M_W_time.png")
    plt.close(fig3)

    # 4) Output Y
    fig4 = plt.figure()
    plt.plot(Ymean, label="Mean Y")
    plt.title(f"Output (quality×efficiency) — {name}")
    plt.xlabel("Time")
    plt.ylabel("Y")
    plt.legend()
    fig4.tight_layout()
    fig4.savefig(out / f"{name}_Y_time.png")
    plt.close(fig4)

def plot_final_distributions(res_a: Dict[str, np.ndarray], name_a: str,
                             res_b: Dict[str, np.ndarray], name_b: str):
    out = ensure_out()
    Ka = res_a["K"][-1]; Kb = res_b["K"][-1]
    fig = plt.figure()
    plt.hist(Ka, bins=30, alpha=0.6, label=f"{name_a} final K")
    plt.hist(Kb, bins=30, alpha=0.6, label=f"{name_b} final K")
    plt.title("Final Knowledge Distributions")
    plt.xlabel("K")
    plt.ylabel("Count")
    plt.legend()
    fig.tight_layout()
    fig.savefig(out / f"final_K_distributions_{name_a}_vs_{name_b}.png")
    plt.close(fig)

# -----------------------------
# Main
# -----------------------------
def main():
    g = GlobalParams()
    np.random.seed(g.seed)

    # Baseline
    base = ScenarioToggles(name="baseline")
    res_base = run_scenario(g, base)

    # Strong scaffolding & personalization, lower hallucination
    pro_eq = ScenarioToggles(
        name="pro_equity",
        S=0.9, P=0.7, H=0.03, rho=0.5
    )
    res_eq = run_scenario(g, pro_eq)

    # High complementarity (risk of Matthew effect), weaker scaffolding
    pro_matthew = ScenarioToggles(
        name="pro_matthew",
        S=0.2, P=0.1, H=0.10, rho=1.0
    )
    res_mat = run_scenario(g, pro_matthew)

    # Plot per-scenario series
    for name, res in [("baseline", res_base), ("pro_equity", res_eq), ("pro_matthew", res_mat)]:
        plot_time_series(res, name)

    # Compare final distributions
    plot_final_distributions(res_eq, "pro_equity", res_mat, "pro_matthew")

    # Print summary table
    def summarize(res, label):
        K = res["K"]; U = res["U"]; G = res["G"]; Y = res["Y"]
        out = dict(
            label=label,
            K_mean_final=float(K[-1].mean()),
            K_p10_final=float(np.percentile(K[-1], 10)),
            K_p90_final=float(np.percentile(K[-1], 90)),
            Gini_final=float(G[-1]),
            U_mean_final=float(U[-1].mean()),
            Y_mean_final=float(Y[-1].mean())
        )
        return out

    rows = [summarize(res_base, "baseline"),
            summarize(res_eq, "pro_equity"),
            summarize(res_mat, "pro_matthew")]
    df = pd.DataFrame(rows)
    print(df.to_string(index=False))

    # Save CSV
    out_dir = ensure_out()
    df.to_csv(out_dir / "summary.csv", index=False)
    print(f"\nSaved figures and summary CSV to: {out_dir.resolve()}")

if __name__ == "__main__":
    main()
