import matplotlib.pyplot as plt
import numpy as np

def plot_trajectory(points, dt=None, filename="otto.png", show=True):
    """
    Mostra la traiettoria: vista 3D, vista dall'alto (x-y) e quota nel tempo.

    points   : lista di tuple (x, y, z), ad esempio la lista `otto`
    dt       : passo temporale tra i punti [s]; se None l'asse orizzontale
               del terzo grafico è l'indice del punto
    filename : se non è None salva l'immagine
    show     : apre la finestra (mettilo a False su una macchina senza display)
    """
    P = np.asarray(points, dtype=float)
    if P.ndim != 2 or P.shape[1] != 3 or len(P) == 0:
        raise ValueError("points deve essere una lista di tuple (x, y, z)")
    x, y, z = P.T
    t = np.arange(len(P)) * dt if dt else np.arange(len(P))

    line, start, end = "#2E6FD8", "#1F9D55", "#C8372D"
    fig = plt.figure(figsize=(15, 5))

    # --- Vista 3D
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    ax.plot(x, y, z, color=line, lw=2)
    ax.scatter(*P[0], color=start, s=40, label="Partenza", depthshade=False)
    ax.scatter(*P[-1], color=end, s=40, marker="s", label="Arrivo", depthshade=False)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_box_aspect(np.maximum(np.ptp(P, axis=0), 1e-3))  # metri uguali sui 3 assi
    ax.set_title("Vista 3D")
    ax.legend(loc="upper left")

    # --- Vista dall'alto
    ax = fig.add_subplot(1, 3, 2)
    ax.plot(x, y, color=line, lw=2)
    ax.plot(x[0], y[0], "o", color=start, ms=8)
    ax.plot(x[-1], y[-1], "s", color=end, ms=8)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Vista dall'alto (x-y)")
    ax.grid(alpha=0.3)

    # --- Quota nel tempo
    ax = fig.add_subplot(1, 3, 3)
    ax.plot(t, z, color=line, lw=2)
    ax.set_xlabel("t [s]" if dt else "indice punto"); ax.set_ylabel("z [m]")
    ax.set_title("Quota")
    ax.grid(alpha=0.3)

    fig.tight_layout()
    if filename:
        fig.savefig(filename, dpi=150)
        print(f"[*] Grafico salvato in {filename}")
    if show:
        plt.show()
    plt.close(fig)
