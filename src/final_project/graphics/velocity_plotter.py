"""
Confronta la velocità stimata (velocity_log.txt) con la velocità "vera"
misurata dal pom (pom.log), limitandosi all'intervallo di tempo comune
ai due file, e produce un'unica immagine con 3 subplot (Vx, Vy, Vz).
 
Uso:
    python3 compare_velocity.py [pom_log_path] [velocity_log_path] [output_path]
 
Se non vengono passati argomenti, usa i default sotto (stessa cartella dello script).
"""
 
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
 
 
def load_pom_log(path):
    """
    Parsa il file pom.log.
    Formato: righe di commento che iniziano con '#', righe vuote da ignorare,
    header con i nomi colonna preceduto da '#', poi righe di dati separate da spazi.
    Colonne di interesse (in base all'header):
        ts vx vy vz  -> indici 0, 13, 14, 15
    """
    timestamps = []
    vx, vy, vz = [], [], []
 
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            # riga di dati valida: deve avere almeno 16 colonne (indice 15 = vz)
            if len(parts) < 16:
                continue
            try:
                ts = float(parts[0])
                v_x = float(parts[13])
                v_y = float(parts[14])
                v_z = float(parts[15])
            except ValueError:
                continue
            timestamps.append(ts)
            vx.append(v_x)
            vy.append(v_y)
            vz.append(v_z)
 
    return (np.array(timestamps), np.array(vx), np.array(vy), np.array(vz))
 
 
def load_velocity_log(path):
    """
    Parsa il file velocity_log.txt.
    Formato: timestamp, vx, vy, vz (separati da virgola)
    """
    timestamps, vx, vy, vz = [], [], [], []
 
    with open(path, "r") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) < 4:
                continue
            try:
                ts, v_x, v_y, v_z = (float(p) for p in parts[:4])
            except ValueError:
                continue
            timestamps.append(ts)
            vx.append(v_x)
            vy.append(v_y)
            vz.append(v_z)
 
    return (np.array(timestamps), np.array(vx), np.array(vy), np.array(vz))
 
 
def clip_to_common_interval(t_a, t_b):
    """Ritorna (t_start, t_end) dell'intervallo di tempo comune a due serie temporali."""
    t_start = max(t_a.min(), t_b.min())
    t_end = min(t_a.max(), t_b.max())
    return t_start, t_end
 
 
def nearest_match(t_ref, t_pom, vx_pom, vy_pom, vz_pom):
    """
    Per ogni timestamp in t_ref (quello che logga meno frequentemente),
    trova il campione pom più vicino nel tempo e ne prende il valore.
    t_pom deve essere ordinato crescente (i log lo sono già per costruzione).
    """
    idx = np.searchsorted(t_pom, t_ref)
    idx = np.clip(idx, 1, len(t_pom) - 1)
 
    left = idx - 1
    right = idx
    # scegli, tra il campione precedente e quello successivo, il più vicino
    use_left = np.abs(t_pom[left] - t_ref) <= np.abs(t_pom[right] - t_ref)
    best_idx = np.where(use_left, left, right)
 
    return vx_pom[best_idx], vy_pom[best_idx], vz_pom[best_idx]
 
 
def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
 
    pom_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(script_dir, "pom.log")
    vel_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(script_dir, "velocity_log.txt")
    out_path = sys.argv[3] if len(sys.argv) > 3 else os.path.join(script_dir, "velocity_comparison.png")
 
    # Carica i due file
    t_pom, vx_pom, vy_pom, vz_pom = load_pom_log(pom_path)
    t_est, vx_est, vy_est, vz_est = load_velocity_log(vel_path)
 
    if len(t_pom) == 0:
        raise RuntimeError(f"Nessun dato valido trovato in {pom_path}")
    if len(t_est) == 0:
        raise RuntimeError(f"Nessun dato valido trovato in {vel_path}")
 
    # Intervallo di tempo comune
    t_start, t_end = clip_to_common_interval(t_pom, t_est)
    print(f"Intervallo comune: [{t_start:.3f}, {t_end:.3f}]  (durata: {t_end - t_start:.2f} s)")
 
    mask_est = (t_est >= t_start) & (t_est <= t_end)
    t_est_c, vx_est_c, vy_est_c, vz_est_c = (t_est[mask_est], vx_est[mask_est],
                                              vy_est[mask_est], vz_est[mask_est])
 
    # Per ogni timestamp di velocity_log (meno frequente), prendo il campione
    # pom più vicino nel tempo, invece di tenere tutti i punti pom
    vx_pom_c, vy_pom_c, vz_pom_c = nearest_match(t_est_c, t_pom, vx_pom, vy_pom, vz_pom)
 
    # Tempo relativo (parte da 0) per leggibilità sull'asse x
    t0 = t_start
    t_pom_rel = t_est_c - t0   # stessi istanti di velocity_log
    t_est_rel = t_est_c - t0
 
    # Plot: 3 subplot verticali, uno per componente
    fig, axes = plt.subplots(3, 1, figsize=(10, 10), sharex=True)
 
    components = [
        ("Vx", vx_pom_c, vx_est_c, axes[0]),
        ("Vy", vy_pom_c, vy_est_c, axes[1]),
        ("Vz", vz_pom_c, vz_est_c, axes[2]),
    ]
 
    for label, pom_data, est_data, ax in components:
        ax.plot(t_pom_rel, pom_data, label="pom (verità)", color="tab:green", linewidth=1.5)
        ax.plot(t_est_rel, est_data, label="stima (velocity_log)", color="tab:red",
                linestyle="none", marker="o", markersize=3)
        ax.set_ylabel(f"{label} [m/s]")
        ax.grid(True, alpha=0.3)
        ax.legend(loc="upper right")
 
    axes[-1].set_xlabel("Tempo [s] (relativo all'inizio dell'intervallo comune)")
    fig.suptitle("Confronto velocità: pom (verità) vs stima (velocity_log)", fontsize=14)
    fig.tight_layout(rect=[0, 0, 1, 0.97])
 
    fig.savefig(out_path, dpi=150)
    print(f"Plot salvato in: {out_path}")
 
 
if __name__ == "__main__":
    main()