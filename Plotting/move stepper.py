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

def arrow(ax, p1, p2, text=None, xo=0.0, yo=0.0, style="->"):
    (x1, y1), (x2, y2) = p1, p2
    ax.annotate("",
        xy=(x2, y2), xytext=(x1, y1),
        arrowprops=dict(arrowstyle=style, lw=1.2, color="#333333"))
    if text:
        ax.text((x1+x2)/2+xo, (y1+y2)/2+yo, text, fontsize=9, va='center', ha='center')

def draw_move_stepper_flow(path='move_stepper_flow.png'):
    mpl.rcParams['figure.dpi'] = 160
    fig, ax = plt.subplots(figsize=(8.5, 11))
    ax.set_xlim(0, 10)
    ax.set_ylim(0, 22)
    ax.axis('off')

    W, H = 6.8, 1.3
    E_W, E_H = 3.2, 1.3
    D_W, D_H = 4.4, 1.8

    y = 21
    start = add_ellipse(ax, 5, y, E_W, E_H, "Start move_stepper(steps, stepPeriod, direction)")
    
    y -= 2.0
    timeout_box = add_box(ax, 5, y, W+1, H, "@timeout_decorator.timeout(0.02, timeout_exception=TimeoutError)", fc="#ffe6e6")
    
    y -= 2.2
    check_dir = add_diamond(ax, 5, y, D_W, D_H, "direction == 1?")
    
    # Direction branches
    set_high = add_box(ax, 2.0, y-2.2, W, H, "GPIO.output(self.DIR, GPIO.HIGH)")
    check_neg = add_diamond(ax, 8.0, y-2.2, D_W, D_H, "direction == -1?")
    set_low = add_box(ax, 8.0, y-4.4, W, H, "GPIO.output(self.DIR, GPIO.LOW)")
    raise_error = add_box(ax, 8.0, y-6.6, W, H, "raise ValueError('Direction must be 1 or -1')", fc="#ffcccc")
    
    # Main flow continues
    y_main = y - 8.0
    start_timer = add_box(ax, 5, y_main, W, H, "startTime = time.perf_counter()")
    
    y_main -= 2.0
    loop_init = add_box(ax, 5, y_main, W, H, "for _ in range(steps):")
    
    y_main -= 2.0
    pulse_high = add_box(ax, 5, y_main, W, H, "GPIO.output(self.pulsePin, GPIO.HIGH)")
    
    y_main -= 2.0
    sleep1 = add_box(ax, 5, y_main, W, H, "time.sleep(stepPeriod/2)")
    
    y_main -= 2.0
    pulse_low = add_box(ax, 5, y_main, W, H, "GPIO.output(self.pulsePin, GPIO.LOW)")
    
    y_main -= 2.0
    sleep2 = add_box(ax, 5, y_main, W, H, "time.sleep(stepPeriod/2)")
    
    y_main -= 2.2
    loop_check = add_diamond(ax, 5, y_main, D_W, D_H, "More steps?")
    
    y_main -= 2.5
    end_timer = add_box(ax, 5, y_main, W, H, "endTime = time.perf_counter()")
    
    y_main -= 2.0
    calc_time = add_box(ax, 5, y_main, W, H, "movementTime = endTime - startTime")
    
    y_main -= 2.0
    return_time = add_box(ax, 5, y_main, W, H, "return movementTime")
    
    y_main -= 2.0
    end = add_ellipse(ax, 5, y_main, E_W, E_H, "End")

    # Main flow arrows
    arrow(ax, start, timeout_box)
    arrow(ax, timeout_box, check_dir)
    
    # Direction handling
    arrow(ax, check_dir, set_high, "Yes", xo=-0.7, yo=0.2)
    arrow(ax, check_dir, check_neg, "No", xo=0.7, yo=0.2)
    arrow(ax, check_neg, set_low, "Yes", xo=0.0, yo=0.2)
    arrow(ax, check_neg, raise_error, "No", xo=0.0, yo=0.2)
    
    # Convergence to main flow
    arrow(ax, set_high, start_timer)
    arrow(ax, set_low, start_timer)
    
    # Main execution flow
    arrow(ax, start_timer, loop_init)
    arrow(ax, loop_init, pulse_high)
    arrow(ax, pulse_high, sleep1)
    arrow(ax, sleep1, pulse_low)
    arrow(ax, pulse_low, sleep2)
    arrow(ax, sleep2, loop_check)
    arrow(ax, loop_check, pulse_high, "Yes", xo=-2.5, yo=0.2)  # Loop back
    arrow(ax, loop_check, end_timer, "No", xo=0.0, yo=0.2)
    arrow(ax, end_timer, calc_time)
    arrow(ax, calc_time, return_time)
    arrow(ax, return_time, end)

    plt.tight_layout()
    plt.savefig(path, bbox_inches='tight')
    plt.close(fig)
    return path

if __name__ == "__main__":
    print("Saved:", draw_move_stepper_flow())
