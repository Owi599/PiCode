# pip install matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Ellipse
import matplotlib as mpl

# Helpers
def add_box(ax, x, y, w, h, text, fc="#f8f8f8"):
    rect = FancyBboxPatch((x - w/2, y - h/2), w, h,
                          boxstyle="round,pad=0.02,rounding_size=0.05",
                          ec="#333333", fc=fc, lw=1.2)
    ax.add_patch(rect)
    ax.text(x, y, text, ha='center', va='center', fontsize=10)
    return (x, y)

def add_diamond(ax, x, y, w, h, text, fc="#fff7e6"):
    pts = [(x, y + h/2), (x + w/2, y), (x, y - h/2), (x - w/2, y)]
    poly = Polygon(pts, closed=True, ec="#333333", fc=fc, lw=1.2)
    ax.add_patch(poly)
    ax.text(x, y, text, ha='center', va='center', fontsize=10)
    return (x, y)

def add_ellipse(ax, x, y, w, h, text, fc="#eef6ff"):
    e = Ellipse((x, y), w, h, ec="#333333", fc=fc, lw=1.2)
    ax.add_patch(e)
    ax.text(x, y, text, ha='center', va='center', fontsize=10)
    return (x, y)

def arrow(ax, p1, p2, text=None, xo=0.0, yo=0.0):
    (x1, y1), (x2, y2) = p1, p2
    ax.annotate("",
        xy=(x2, y2), xytext=(x1, y1),
        arrowprops=dict(arrowstyle="->", lw=1.2, color="#333333"))
    if text:
        ax.text((x1+x2)/2+xo, (y1+y2)/2+yo, text, fontsize=9, va='center', ha='center')

def draw_main_loop_flow(path='main_loop_flow.png'):
    mpl.rcParams['figure.dpi'] = 160
    fig, ax = plt.subplots(figsize=(12, 13))
    ax.set_xlim(0, 18)
    ax.set_ylim(0, 26)
    ax.axis('off')

    W, H = 6.8, 1.3
    E_W, E_H = 3.2, 1.3
    D_W, D_H = 4.4, 1.8

    y = 24
    start = add_ellipse(ax, 9, y, E_W, E_H, "Start main loop")
    y -= 2.0
    read = add_box(ax, 9, y, W, H, "read_sensors_data()")
    y -= 2.0
    ctrl = add_box(ax, 9, y, W, H, "compute_controller_output(K, sensorData)")
    y -= 2.0
    steps = add_box(ax, 9, y, W, H, "calculate_steps(u)")
    y -= 2.0
    move = add_box(ax, 9, y, W, H, "move_stepper(steps, stepPeriod, dir, timeout=0.02)")
    y -= 2.0
    logt = add_box(ax, 9, y, W, H, "append timings (sensor, control, step, movement)")
    y -= 2.0
    it = add_ellipse(ax, 9, y, 1.2, 1.2, "next iteration", fc="#ffffff")

    # Normal flow
    arrow(ax, start, read, "while True", yo=0.5)
    arrow(ax, read, ctrl)
    arrow(ax, ctrl, steps)
    arrow(ax, steps, move)
    arrow(ax, move, logt)
    arrow(ax, logt, it)
    arrow(ax, it, read)

    # Exceptions cluster
    base_y = 18
    to_err = add_diamond(ax, 3, base_y, D_W, D_H, "TimeoutError")
    kbi_err = add_diamond(ax, 3, base_y-3.5, D_W, D_H, "KeyboardInterrupt")
    gen_err = add_diamond(ax, 3, base_y-7.0, D_W, D_H, "Exception")

    to_act = add_box(ax, 3, base_y-1.5, W, H, "print(\"Function timed out...\"); continue")
    kbi_cleanup = add_box(ax, 3, base_y-5.0, W, H, "GPIO.cleanup(); save CSVs; print status")
    gen_cleanup = add_box(ax, 3, base_y-8.5, W, H, "print error; GPIO.cleanup(); save CSVs")

    end = add_ellipse(ax, 3, base_y-10.3, E_W, E_H, "End")

    # Dashed branches from main nodes
    # for p in [read, ctrl, steps, move, logt]:
    #     arrow(ax, p, to_err, "except TimeoutError", xo=-0.9, yo=0.3)
    #     arrow(ax, p, kbi_err, "except KeyboardInterrupt", xo=-0.9, yo=0.3)
    #     arrow(ax, p, gen_err, "except Exception", xo=-0.9, yo=0.3)
    # Convert those arrows to dashed by overlaying thinner lines (simple effect)
    # (Matplotlib annotate doesn't directly support dashed arrows; could use FancyArrowPatch if desired)

    # Exception flows
    arrow(ax, to_err, to_act)
    arrow(ax, to_act, it, "continue")

    arrow(ax, kbi_err, kbi_cleanup)
    arrow(ax, kbi_cleanup, end)

    arrow(ax, gen_err, gen_cleanup)
    arrow(ax, gen_cleanup, end)

    # GPIO interrupts cluster (optional)
    gx, gy = 15, 18
    int1 = add_box(ax, gx, gy, W, H, "FALLING on switchPin_1", fc="#e7f3ff")
    int2 = add_box(ax, gx, gy-2.0, W, H, "FALLING on switchPin_2", fc="#e7f3ff")
    cb = add_box(ax, gx, gy-4.0, W, H, "end_switch_callback(): MotorControl.stop_motor()", fc="#e7f3ff")
    arrow(ax, int1, cb)
    arrow(ax, int2, cb)

    plt.tight_layout()
    plt.savefig(path, bbox_inches='tight')
    plt.close(fig)
    return path

if __name__ == "__main__":
    print("Saved:", draw_main_loop_flow())
