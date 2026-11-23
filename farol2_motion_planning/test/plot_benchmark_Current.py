#!/usr/bin/env python3
"""
plot_control_points.py

Reads the control-point CSV produced by benchmark_planner.cpp and generates
one figure for each Bezier degree.

Each figure contains the control points and connecting polygons for all
vehicles and all successful trials corresponding to that degree.

Usage:
    python3 plot_control_points.py [--indir DIR] [--outdir DIR] [--date YYYY-MM-DD]

Requires: pandas, matplotlib
"""

import argparse
import datetime
import os

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np


RESULTS_DIR = "/home/dsor-gorka/dsor/colcon_ws_mDrivers/src/farol2/farol2_motion_planning/test/results"


def plot_control_points(csv_path_cp, csv_path_traj, outdir, degrees=None):

    if not os.path.exists(csv_path_cp):
        print(f"[skip] {csv_path_cp} not found")
        return
    base = os.path.splitext(os.path.basename(csv_path_cp))[0]

    if not os.path.exists(csv_path_traj):
        print(f"[skip] {csv_path_traj} not found")
        return

    df = pd.read_csv(csv_path_cp)
    df_traj = pd.read_csv(csv_path_traj)

    required_cp = {"label", "bezier_degree", "trial", "vehicle", "control_point", "x", "y"}
    required_traj = {"label", "bezier_degree", "trial", "vehicle", "point_idx", "x", "y", "tf"}
    

    missing = required_cp - set(df.columns)
    if missing:
        raise ValueError(f"Control-point CSV is missing: {sorted(missing)}")

    missing = required_traj - set(df_traj.columns)
    if missing:
        raise ValueError(f"Trajectory CSV is missing: {sorted(missing)}")

    markers = ["o", "s", "^", "D", "v", "<", ">", "P", "X", "*"]

    available_degrees = sorted(df["bezier_degree"].unique())

    fig = plt.figure(figsize=(7, 9))
    gs = fig.add_gridspec(2, 1, height_ratios=[1, 3.5], hspace=0.25)
    ax_cur = fig.add_subplot(gs[0])
    ax = fig.add_subplot(gs[1])

    fig_vel, ax_vel = plt.subplots(figsize=(7, 5))
    fig_heading, ax_heading = plt.subplots(figsize=(7, 5))

    if degrees is None:
        degrees_to_plot = available_degrees
    else:
        degrees_to_plot = [d for d in degrees if d in available_degrees]

    for degree in degrees_to_plot:

        degree_df = df[df["bezier_degree"] == degree]
        degree_traj = df_traj[df_traj["bezier_degree"] == degree]

        

        for vehicle in sorted(degree_df["vehicle"].unique()):

            marker = markers[vehicle % len(markers)]

            vehicle_df = degree_df[degree_df["vehicle"] == vehicle]
            vehicle_traj = degree_traj[degree_traj["vehicle"] == vehicle]
            first_trial = True

            for trial in sorted(vehicle_df["trial"].unique()):

                cp_trial = vehicle_df[vehicle_df["trial"] == trial].sort_values("control_point")
                traj_trial = vehicle_traj[vehicle_traj["trial"] == trial].sort_values("point_idx")
                Tf = traj_trial["tf"].iloc[0]

                t = np.linspace(0.0, Tf, len(traj_trial))

                x = traj_trial["x"].to_numpy()
                y = traj_trial["y"].to_numpy()

                dx_dt = np.gradient(x, t) + y
                dy_dt = np.gradient(y, t)

                velocity = np.sqrt(dx_dt**2 + dy_dt**2)
                heading = np.degrees(np.unwrap(np.arctan2(dy_dt, dx_dt)))

                # ------------------------------------------------
                # Label
                # ------------------------------------------------
                #
                # We want the legend to identify the BEZIER
                # DEGREE, not every trial.
                #
                label = f"Bézier degree {degree}" if first_trial else None


                ax_vel.plot(t,velocity,linewidth=1.0,alpha=0.5,label=label,)

                ax_heading.plot(t,heading,linewidth=1.0,alpha=0.5,label=label,)
                # Trajectory
                ax.plot(traj_trial["y"], traj_trial["x"], linewidth=1.0, alpha=0.5, label=label)

                if first_trial:
                    x0, y0 = cp_trial["x"].iloc[0], cp_trial["y"].iloc[0]
                    xf, yf = cp_trial["x"].iloc[-1], cp_trial["y"].iloc[-1]
                    ax.plot(y0, x0, marker="o", color="black", markersize=5, zorder=5)
                    ax.annotate(r"$\mathbf{P}_0$", (y0, x0), textcoords="offset points", xytext=(-12, -14), fontsize=14)
                    ax.plot(yf, xf, marker="o", color="black", markersize=5, zorder=5)
                    ax.annotate(r"$\mathbf{P}_f$", (yf, xf), textcoords="offset points", xytext=(6, -12), fontsize=14)

                first_trial = False

    theta0_analytical = np.deg2rad(105.0190979013)
    thetaf_analytical = np.deg2rad(239.9818466752)

    tan_theta0 = np.tan(theta0_analytical)

    # Dimensionless final time V*t_f/h
    tau_f_analytical = (
        np.tan(thetaf_analytical) - tan_theta0
    )

    # If your t is already dimensionless V*t/h:
    tau_analytical = np.linspace(0.0, tau_f_analytical, 1000)

    # Continuous heading branch:
    theta_analytical = (
        np.pi
        + np.arctan(tan_theta0 + tau_analytical)
    )

    y_analytical = 1/np.cos(theta_analytical) - 1/np.cos(thetaf_analytical)
    x_analytical = (
        (-1/np.cos(thetaf_analytical)) * (np.tan(thetaf_analytical) - np.tan(theta_analytical))
        + np.tan(theta_analytical) * (1/np.cos(thetaf_analytical) - 1/np.cos(theta_analytical))
        + np.log(
            (np.tan(theta_analytical) + 1/np.cos(theta_analytical)) /
            (np.tan(thetaf_analytical) + 1/np.cos(thetaf_analytical))
        )
    ) / 2


    normal_size = 14
    smaller_size = 13
    theta_analytical_deg = np.rad2deg(theta_analytical)
    
    
    ax.set_xlabel("y [m]", fontsize=normal_size)
    ax.set_ylabel("x [m]", fontsize=normal_size)
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, linewidth=0.4, alpha=0.5)
    ax.plot(y_analytical, x_analytical, "--", label="Analytical", linewidth=2)

    # Heading arrows: heading theta is measured from x, so the horizontal (y) component is sin and the vertical (x) component is cos
    n_arrows = 12
    k = np.linspace(0, len(x_analytical) - 1, n_arrows).astype(int)
    L = 0.25
    ax.quiver(y_analytical[k], x_analytical[k],
              L*np.sin(theta_analytical[k]), L*np.cos(theta_analytical[k]),
              angles="xy", scale_units="xy", scale=1,
              width=0.004, color="k", zorder=6)


    # ---------------------------- Little boat ----------------------------
        # ---- Velocity triangle and boat at P0 ----
    # P0 in plot coordinates (horizontal = y, vertical = x)
    px, py = y_analytical[0], x_analytical[0]
    th0 = theta_analytical[0]
    s = 1.0                                         # scale of the vectors in data units

    V_vec  = s*np.array([np.sin(th0), np.cos(th0)])   # relative to water (horizontal, vertical)
    c_vec  = s*np.array([0.0, -y_analytical[0]])      # current u/V = -y/h, along x (vertical)
    R_vec  = V_vec + c_vec                            # resultant (ground) velocity

    # Small boat at P0, pointing along the heading
    h = np.array([np.sin(th0), np.cos(th0)])
    n = np.array([np.cos(th0), -np.sin(th0)])
    b = 0.22
    boat_local = np.array([[ 0.6,  0.0],
                           [ 0.2,  0.25],
                           [-0.5,  0.25],
                           [-0.5, -0.25],
                           [ 0.2, -0.25]])
    boat = np.array([px, py]) + b*(boat_local[:, [0]]*h + boat_local[:, [1]]*n)
    ax.add_patch(plt.Polygon(boat, closed=True, facecolor="white",
                             edgecolor="k", linewidth=1.2, zorder=7))

    def vec(origin, v, color, lw=1.4, z=8):
        ax.annotate("", xy=origin + v, xytext=origin, annotation_clip=False,
                    arrowprops=dict(arrowstyle="-|>", lw=lw, color=color), zorder=z)

    p0 = np.array([px, py])
    vec(p0,          V_vec, "k")                     # V: velocity relative to water
    vec(p0 + V_vec,  c_vec, "tab:gray")              # current, placed at the tip of V
    vec(p0,          R_vec, "tab:red")               # resultant velocity

    ax.update_datalim([p0, p0 + V_vec, p0 + V_vec + c_vec, p0 + R_vec])
    ax.autoscale_view()
    ax.annotate(r"$\mathbf{V}$", xy=p0 + 0.5*V_vec, xytext=(2, -10), textcoords="offset points",
                fontsize=smaller_size, ha="center", va="center", zorder=9)
    ax.annotate(r"$\mathbf{u}_c$", xy=p0 + V_vec + 0.5*c_vec, xytext=(12, 0), textcoords="offset points",
                fontsize=smaller_size, color="tab:gray", ha="center", va="center", zorder=9)
    # ax.annotate(r"$\mathbf{V}+\mathbf{v}_c$", xy=p0 + 0.5*R_vec, xytext=(-25, -8), textcoords="offset points",
    #             fontsize=smaller_size, color="tab:red", ha="center", va="center", zorder=9)
    # ---------------------------- ----------- ----------------------------

    ax.tick_params(axis="both", labelsize=smaller_size)
    ax.legend(fontsize=smaller_size)

    # Let the layout settle, then read the final horizontal (y) range of the map
    fig.tight_layout()
    fig.canvas.draw()
    ymin, ymax = ax.get_xlim()

    # Current panel on top: u/V = -y/h, arrows along x (vertical), same horizontal range as the map
    ax_cur.set_xlim(ymin, ymax)
    ax_cur.set_ylim(-2.6, 2.6)
    ax_cur.plot([ymin, ymax], [-ymin, -ymax], "k", linewidth=1.5)    # profile line
    for yk in np.linspace(ymin, ymax, 13):
        if abs(yk) < 1e-9:
            continue
        ax_cur.annotate("", xy=(yk, -yk), xytext=(yk, 0),
                        arrowprops=dict(arrowstyle="-|>", lw=1.5, color="k"))

    ax_cur.spines["bottom"].set_position(("data", 0))   # horizontal line at u/V = 0
    for s in ("top", "right"):
        ax_cur.spines[s].set_visible(False)
    ax_cur.set_xticks([])
    ax_cur.set_yticks([-2, -1, 1, 2])
    ax_cur.tick_params(axis="y", labelsize=smaller_size - 3)
    ax_cur.set_ylabel(r"$\mathbf{u}_c$", fontsize=normal_size)
    ax_cur.set_title("Current", fontsize=smaller_size)

    filename = os.path.join(outdir, f"{base}_traj.png")
    fig.savefig(filename, dpi=300, bbox_inches="tight")

    print(f"Saved {filename}")
    plt.close(fig)

    ax_vel.set_xlabel("Time [s]", fontsize=normal_size)
    ax_vel.set_ylabel("Velocity magnitude", fontsize=normal_size)
    ax_vel.grid(True, linewidth=0.4, alpha=0.5)
    ax_vel.tick_params(axis="both", labelsize=smaller_size)
    #ax_vel.legend(fontsize=smaller_size)
    ax_vel.set_ylim(0.0, 1.1)

    fig_vel.tight_layout()
    
    filename = os.path.join(outdir, f"{base}_vel.png")
    fig_vel.savefig(filename, dpi=300, bbox_inches="tight")

    print(f"Saved {filename}")
    plt.close(fig_vel)

    ax_heading.set_xlabel("Time [s]", fontsize=normal_size)
    ax_heading.set_ylabel("Heading [ º]", fontsize=normal_size)
    ax_heading.grid(True, linewidth=0.4, alpha=0.5)
    ax_heading.plot(tau_analytical, theta_analytical_deg,"--",label="Analytical",linewidth=2)
    ax_heading.tick_params(axis="both", labelsize=smaller_size)
    ax_heading.legend(fontsize=smaller_size)
    # Analytical solution
    
    ax_heading.set_ylim(95, 245)



    fig_heading.tight_layout()
            
    filename = os.path.join(outdir, f"{base}_head.png")
    fig_heading.savefig(filename, dpi=300, bbox_inches="tight")

    print(f"Saved {filename}")
    plt.close(fig_heading)


def main():

    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--indir",
        default=RESULTS_DIR,
        help="directory containing the control-point CSV",
    )

    parser.add_argument(
        "--outdir",
        default=os.path.join(RESULTS_DIR, "figures"),
        help="directory to write PNG figures to",
    )

    parser.add_argument(
        "--date",
        default=datetime.date.today().isoformat(),
        help="date tag matching the CSV filename, format YYYY-MM-DD (default: today)",
    )

    parser.add_argument(
        "--degrees",
        nargs="+",
        type=int,
        default=None,
        help="Bezier degrees to plot (default: all degrees present in the CSV)",
    )

    args = parser.parse_args()

    indir = os.path.join(RESULTS_DIR, args.date)
    outdir = os.path.join(indir, "figures")
    os.makedirs(outdir, exist_ok=True)

    plot_control_points(
        csv_path_cp=os.path.join(indir, f"benchmark_{args.date}_controlPts.csv"),
        csv_path_traj=os.path.join(indir, f"benchmark_{args.date}_trajectories.csv"),

        outdir=outdir,
        degrees=args.degrees,
    )


if __name__ == "__main__":
    main()