import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, Polygon, Ellipse
import matplotlib as mpl

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

def draw_calculate_steps_flow(path='calculate_steps_flow.png'):
    mpl.rcParams['figure.dpi'] = 160
    fig, ax = plt.subplots(figsize=(8.5, 11))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 22)
    ax.axis('off')

    W, H = 6.4, 1.3
    E_W, E_H = 3.2, 1.3
    D_W, D_H = 4.4, 1.8

    y = 21
    S = add_ellipse(ax, 5, y, E_W, E_H, "Start calculate_steps(force)")
    y -= 2.0
    A = add_box(ax, 5, y, W, H, "startTime = perf_counter()")
    y -= 2.0
    B = add_box(ax, 5, y, W, H, "acceleration = force / m_total")
    y -= 2.0
    C = add_box(ax, 5, y, W, H, "velocity_integral += acceleration * dt")
    y -= 2.0
    D = add_box(ax, 5, y, W, H, "position_integral += velocity_integral * dt")
    y -= 2.2
    E = add_diamond(ax, 5, y, D_W, D_H, "|velocity_integral| > velocity_max ?")

    F = add_box(ax, 2.0, y-2.2, W, H, "velocity_integral = sign(vel) * velocity_max")
    G = add_box(ax, 5, y-4.2, W, H, "steps_int = max(int((pos*stepsPerRev*microres)/(2π*pulleyRad)), 1)")
    Hn = add_box(ax, 5, y-6.2, W, H, "stepFreq = |velocity_integral| * microres / (pulleyRad/(2π))")
    I = add_diamond(ax, 5, y-8.4, D_W, D_H, "stepFreq == 0 ?")
    J1 = add_box(ax, 2.0, y-10.6, W, H, "stepPeriod = 0")
    J2 = add_box(ax, 8.0, y-10.6, W, H, "stepPeriod = 1 / stepFreq")
    K = add_box(ax, 5, y-12.6, W, H, "maxDelay = t_sample")
    L = add_box(ax, 5, y-14.6, W, H, "stepPeriod = min(stepPeriod, maxDelay)")
    M = add_box(ax, 5, y-16.6, W, H, "endTime = perf_counter(); stepCalculationTime = endTime - startTime")
    N = add_box(ax, 5, y-18.6, W, H, "direction = sign(velocity_integral)")
    R = add_box(ax, 5, y-20.6, W, H, "return steps_int, stepPeriod, stepCalculationTime, direction")
    T = add_ellipse(ax, 5, y-22.0, E_W, E_H, "End")

    # Edges
    arrow(ax, S, A); arrow(ax, A, B); arrow(ax, B, C); arrow(ax, C, D); arrow(ax, D, E)
    arrow(ax, E, F, "True", xo=-0.7, yo=0.2); arrow(ax, F, G)
    arrow(ax, E, G, "False", xo=0.7, yo=0.2)
    arrow(ax, G, Hn); arrow(ax, Hn, I)
    arrow(ax, I, J1, "True", xo=-0.7, yo=0.2); arrow(ax, I, J2, "False", xo=0.7, yo=0.2)
    arrow(ax, J1, K); arrow(ax, J2, K)
    arrow(ax, K, L); arrow(ax, L, M); arrow(ax, M, N); arrow(ax, N, R); arrow(ax, R, T)

    plt.tight_layout()
    plt.savefig(path, bbox_inches='tight')
    plt.close(fig)
    return path

if __name__ == "__main__":
    print("Saved:", draw_calculate_steps_flow())
