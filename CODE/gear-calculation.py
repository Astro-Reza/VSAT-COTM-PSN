# digunakan untuk menghitung gear jika diketahui jarak antar center gear dan rasio yang dibutuhkan

# code akan menghitung rasio yang paling terdekat/feasible untuk dibuat.

def estimate_ratios_with_module_match():
    import math

    # input
    r_pinion = float(input("pitch radius pinionnya (mm): "))
    r_gear = float(input("pitch radius gear (mm): "))
    target_ratio = float(input("gear ratio (e.g. 6 for 1:6): "))
    max_error = float(input("ratio error (e.g. 1.0): "))
    tolerance = 0.02  # acceptablenya brp
    results = []

    # kalkulasi
    for z1 in range(12, 35):  # PINION
        for z2 in range(30, 250):  # GEAR 
            ratio = z2 / z1
            error = abs(ratio - target_ratio)

            if error > max_error:
                continue  # sekip kalo rasionya jauh

            # itung
            m1 = (2 * r_pinion) / z1
            m2 = (2 * r_gear) / z2

            # kalo deket ok (toleransi)
            if abs(m1 - m2) <= tolerance:
                avg_m = (m1 + m2) / 2
                results.append({
                    "z1": z1,
                    "z2": z2,
                    "ratio": ratio,
                    "module": avg_m,
                    "error": error
                })

    # Sort results by how close ratio is to target
    results.sort(key=lambda x: x["error"])

    # --- Output ---
    if not results:
        print("⚠️  No gear ratios found within the allowed error range.")
    else:
        print(f"{'Pinion':>7} {'Gear':>6} {'Ratio':>8} {'Module':>8} {'Error':>8}")
        print("-" * 45)
        for r in results:
            print(f"{r['z1']:>7} {r['z2']:>6} {r['ratio']:>8.3f} {r['module']:>8.3f} {r['error']:>8.3f}")

if __name__ == "__main__":
    estimate_ratios_with_module_match()
