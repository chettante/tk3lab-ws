"""
Configurazione centrale del progetto.

Modifica i valori qui sotto e salva: al prossimo avvio di main_statemachine.py
(e dei plotter in graphics/) vengono caricati automaticamente.

Percorsi:
  - i nomi di file *relativi* (es. 'pom.log') sono presi rispetto a 'output_dir'
  - i percorsi *assoluti* (es. '/home/utente/run1/pom.log') sono usati così come sono
  - '~' viene espanso; le cartelle mancanti vengono create
"""

import os

PROJECT_ROOT = os.path.dirname(os.path.abspath(__file__))

CONFIG = {
    # ------------------------------------------------------------------
    # FOV camera (pinhole, come in Gazebo: sync_camera_fov.py copia questi valori nel model.sdf)
    # ------------------------------------------------------------------
    # semi-aperture = angolo tra l'asse ottico e le facce laterali (h) / superiore-inferiore (v)
    'fov_half_angle_h_deg': 43.5,   # orizzontale (sinistra/destra)
    'fov_half_angle_v_deg': 29.0,  # verticale (su/giù)
    'fov_max_range': 5.0,          # m, piano far (profondità lungo l'asse ottico)
    'camera_yaw_deg': 45.0,         # montaggio camera: yaw rispetto all'asse x del drone
                                    # (+45° = bisettrice tra braccio 1 anteriore/rosso e braccio 2)
    'camera_gimbal': False,         # True = camera stabilizzata (solo yaw),
                                    # False = camera rigida sul corpo (roll/pitch/yaw), come in Gazebo

    # ------------------------------------------------------------------
    # Tracking controller, leader centrato nel FOV
    # ------------------------------------------------------------------
    # P + feedforward: v = v_leader + kp·(target − p), target = follow_distance
    # dietro al leader lungo il bearing. Limiti di velocità/yaw: max_velocity e
    # max_yaw_rate (sezione nhfc), uguali per i due droni.
    'tracking_kp_xy': 0.3,
    'tracking_kp_z': 0.6,
    'tracking_follow_distance': 0.0,  # m, distanza orizzontale dal leader (lungo il bearing);
                                      # più grande = il bearing ruota più lentamente quando il leader curva
    'tracking_cmd_acc_max': 1.0,    # m/s², max variazione del comando di velocità (anti-scatto)
    'tracking_kp_angle': 1.5,       # 1/s: ritardo di yaw a regime ≈ velocità del bearing / kp_angle
    'tracking_max_acc': 3.0,        # m/s², passato a maneuver con il comando di velocità
    # priorità allo yaw: la correzione di posizione (xy) cala linearmente da 100% a
    # |errore yaw| = yaw_priority_start fino a 0% al bordo del FOV (fov_half_angle_h_deg)
    'tracking_yaw_priority_start_deg': 25.0,

    # ------------------------------------------------------------------
    # State machine timeouts
    # ------------------------------------------------------------------
    'idle_timeout': 10.0,
    'tracking_timeout': 120.0,
    'out_of_fov_timeout': 0.1,      # grace period con predizione

    # ------------------------------------------------------------------
    # Loop di controllo e logging a terminale
    # ------------------------------------------------------------------
    'control_loop_period': 0.05,
    'log_level': 'INFO',            # livelli: DEBUG, INFO, WARNING, ERROR

    # ------------------------------------------------------------------
    # Controllo di volo di basso livello (nhfc), IDENTICO per leader e follower
    # ------------------------------------------------------------------
    # Kp* posizione, Kv* velocità, Kq* assetto (xy = roll/pitch, z = yaw),
    # Kw* velocità angolare, Ki* integrali. Yaw: ψ'' = -Kqz·e - Kwz·e'
    # (valori originali: 1.0/1.4 sullo yaw ha peggiorato il run, forse sottraendo
    # margine ai motori per la traslazione)
    'nhfc_gains': {
        'Kpxy': 5, 'Kpz': 5, 'Kqxy': 4, 'Kqz': 0.1,
        'Kvxy': 6, 'Kvz': 6, 'Kwxy': 1, 'Kwz': 0.1,
        'Kixy': 0, 'Kiz': 0,
    },
    # limiti dei droni: impostati su maneuver per entrambi e usati per saturare il comando del follower
    'max_velocity': 10.0,            # m/s
    'max_yaw_rate': 2.0,            # rad/s

    # ------------------------------------------------------------------
    # Droni / OptiTrack
    # ------------------------------------------------------------------
    'leader_body': 'QR4_leading',
    'follower_body': 'QR4_following',

    # ------------------------------------------------------------------
    # Leader: posizione iniziale e traiettoria a otto
    # ------------------------------------------------------------------
    'leader_start_pos': (-1.0, 0.0, 1.0),   # goto iniziale (x, y, z) [m]
    'leader_start_yaw': 0.0,                # rad
    'leader_start_velocity': (0.0, 0.0, 0.0),  # m/s just to istantiate the searching class
    'leader_start_duration': 0.0,           # s (0 = il più veloce possibile)

    'leader_random_trajectory': False,       # TEMP: waypoint casuali (random_trajectory) invece dell'otto
    'random_traj_points': 3,                # numero di waypoint casuali (15 s ciascuno + 10 s finale)

    'traj_duration': 40.0,                  # s, tempo in cui il leader completa l'otto
    'traj_tail_time': 15.0,                 # s, il main continua ancora per questo tempo dopo la fine dell'otto
    'traj_A': 5.0,                          # ampiezza lungo y [m]
    'traj_B': 5.0,                          # ampiezza lungo x (semi-asse = B/2) [m]
    'traj_H': 5.0,                          # quota massima sopra P0 [m]
    'traj_P0': (-1.0, 0.0, 1.0),            # punto di partenza/arrivo (di norma = leader_start_pos)

    # ------------------------------------------------------------------
    # Follower: Research strategy parameters
    # ------------------------------------------------------------------
    'traj_duration_research' : 70.0,
    'follower_start_pos': (0.0, 0.0, 0.0),
    'follower_start_yaw': 0.0,

    # recupero: un tratto verso la posizione PREVISTA del leader
    # p_pred = p_last + v_last·(t_lost + T): ruota per centrarlo, si avvicina
    # se è oltre approach_dist, va alla sua quota prevista
    'search_T_rec': 0.5,            # s, durata minima del recupero e orizzonte di previsione
    'search_approach_dist': 3.0,    # m, distanza orizzontale dal leader previsto (< fov_max_range)
    'search_z_min': 0.5,            # m, limiti di quota nel recupero
    'search_z_max': 12.0,
    # scansione globale: home, salita di delta_z_global ruotando, discesa ruotando
    'search_home': (0.0, 0.0, 1.0),
    'search_delta_z_global': 10.0,  # m
    'search_v_scan': 1.0,           # m/s, velocità di picco in salita/discesa
    'search_omega_scan': 0.5,       # rad/s, rotazione durante la scansione
    'search_t_hold': 1.0,           # s, durata delle attese da fermo
    'search_a_max': 1.5,            # m/s², accelerazione di picco dei tratti di ricerca
    'search_yaw_acc_max': 2.0,      # rad/s², accelerazione angolare di picco nel recupero


    # ------------------------------------------------------------------
    # Percorsi: log e immagini
    # ------------------------------------------------------------------
    'output_dir': os.path.join(PROJECT_ROOT, 'graphics'),

    # log salvati da main_statemachine.py (relativi a output_dir)
    'pom_leader_log': 'log/pom.log',
    'pom_follower_log': 'log/pom_f.log',     # GenoM accetta path di max 64 caratteri
    'velocity_log': 'log/velocity_log.txt',

    # immagini (relative a output_dir)
    'plot_planned_trajectory': 'plot/otto.png',
    'plot_real_trajectory': 'plot/real_pos.png',
    'plot_velocity': 'plot/velocity_comparison.png',
    'plot_tracking': 'plot/tracking_analysis.png',
    'show_plots': True,             # False su macchine senza display

    # cartella dei log interni dei componenti GenoM3 (pom/rotorcraft/nhfc/maneuver/optitrack)
    'genom_log_dir': '/tmp',
}


# ============================================================================
# Helper per i percorsi (non serve modificarli)
# ============================================================================

def resolve_output(path):
    """Percorso assoluto di `path`: se relativo è preso rispetto a output_dir.
    Crea la cartella di destinazione se manca."""
    p = os.path.expanduser(str(path))
    if not os.path.isabs(p):
        p = os.path.join(os.path.expanduser(CONFIG['output_dir']), p)
    os.makedirs(os.path.dirname(p), exist_ok=True)
    return p


def config_path(key):
    """Percorso assoluto del file indicato da CONFIG[key] (vedi resolve_output)."""
    return resolve_output(CONFIG[key])


def genom_log_path(filename):
    """Percorso di un log interno GenoM3 dentro 'genom_log_dir'."""
    d = os.path.expanduser(CONFIG['genom_log_dir'])
    os.makedirs(d, exist_ok=True)
    return os.path.join(d, filename)