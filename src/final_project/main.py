#!/usr/bin/env python3
"""
Main script per demo FOV camera e tracking con controllo angolare
Legge posizioni E orientamenti da OptiTrack.bodies
"""

import time
import numpy as np
from functions import *
from camera import *


def read_follower_yaw_from_mocap(follower_body):
    """
    Legge l'orientamento (yaw) del follower da mocap.
    
    OptiTrack fornisce l'orientamento come quaternione nella forma:
    follower_body['bodies']['ori']['w', 'x', 'y', 'z']
    
    Converte a yaw (rotazione intorno a Z)
    """
    try:
        # Leggi quaternione da mocap

        qw = follower_body['bodies']['att']['qw']
        qx = follower_body['bodies']['att']['qx']
        qy = follower_body['bodies']['att']['qy']
        qz = follower_body['bodies']['att']['qz']
        
        # Converti quaternione a yaw (roll-pitch-yaw)
        # yaw = atan2(2*(qw*qz + qx*qy), 1 - 2*(qy^2 + qz^2))
        yaw = np.arctan2(
            2 * (qw * qz + qx * qy),
            1 - 2 * (qy**2 + qz**2)
        )
        
        return yaw
    
    except Exception as e:
        logger.warning(f"Could not read yaw from mocap: {e}, using 0.0")
        return 0.0


def main():
    """Main loop per demo FOV + tracking con controllo angolare"""
    
    print("[*] Setup...")
    setup()
    
    print("[*] Start...")
    start()
    
    # Manda leader a una posizione di test
    print("[*] Sending leader to test position...")
    random_trajectory(n_points=1)
    
    time.sleep(2)  # Aspetta che il leader raggiunga la posizione
    
    # Inizializza FOV camera e tracker
    print("[*] Initializing FOV camera and tracker...")
    camera = FOVPyramid(half_angle_deg=45, max_range=5.0)
    tracker = TrackingController(
        follower=maneuver_f,
        kp=0.8,  # Guadagno distanza
        velocity_estimator=VelocityEstimator(window_size=5, alpha=0.7)
    )
    
    # Imposta limiti di velocità
    vmax = 3.0  # m/s (ridotto da 10.0 per evitare oscillazioni)
    maneuver_f.set_velocity_limit(vmax, 1)
    maneuver_l.set_velocity_limit(1.0, 1)
    
    print("[*] Starting main loop (period: 0.1s, 10Hz)...")
    print("=" * 80)
    print("Reading positions AND orientations from OptiTrack")
    print("Dual control: distance + angular alignment")
    print("=" * 80)
    
    PERIOD = 0.1  # 10 Hz
    next_t = time.monotonic()
    loop_count = 0
    max_loops = 3000  
    
    # Statistiche
    in_fov_count = 0
    out_fov_count = 0
    
    try:
        while loop_count < max_loops:
            # ===== STEP 1: Leggi posizioni E orientamenti da OptiTrack =====
            try:
                leader_body = optitrack.bodies('QR4_leading')
                pos_leader = np.array([
                    leader_body['bodies']['pos']['x'],
                    leader_body['bodies']['pos']['y'],
                    leader_body['bodies']['pos']['z']
                ])
                
                follower_body = optitrack.bodies('QR4_following')
                pos_follower = np.array([
                    follower_body['bodies']['pos']['x'],
                    follower_body['bodies']['pos']['y'],
                    follower_body['bodies']['pos']['z']
                ])
                
                # ✓ NUOVO: Leggi anche orientamento del follower
                follower_yaw = read_follower_yaw_from_mocap(follower_body)
            
            except Exception as e:
                print(f"[{loop_count:03d}] [!] Error reading OptiTrack: {e}")
                next_t += PERIOD
                sleep = next_t - time.monotonic()
                if sleep > 0:
                    time.sleep(sleep)
                else:
                    next_t = time.monotonic()
                loop_count += 1
                continue
            
            # ===== STEP 2: Calcola posizione relativa nel body frame =====
            pos_in_body_frame = camera.global_to_body(pos_leader, pos_follower, follower_yaw)
            distance = np.linalg.norm(pos_in_body_frame)
            
            # ===== STEP 3: Controlla se leader è nel FOV =====
            is_in_fov = camera.contains(pos_in_body_frame)
            
            # ===== STEP 4: Se nel FOV, calcola e invia comando =====
            if is_in_fov:
                in_fov_count += 1
                print(f"\n[{loop_count:03d}] ✓ LEADER IN FOV")
                print(f"      Leader pos: [{pos_leader[0]:7.2f}, {pos_leader[1]:7.2f}, {pos_leader[2]:7.2f}]")
                print(f"      Follower pos: [{pos_follower[0]:7.2f}, {pos_follower[1]:7.2f}, {pos_follower[2]:7.2f}]")
                print(f"      Distance: {distance:6.2f}m")
                print(f"      Follower yaw: {np.degrees(follower_yaw):7.1f}°")
                
                # Aggiorna stima velocità del leader
                leader_vel = tracker.velocity_estimator.update(pos_leader)
                
                # ✓ CORRETTO: Passa anche follower_yaw al controller
                velocity_cmd, yaw_cmd = tracker.compute_command_with_angular_control(
                    leader_pos=pos_leader,
                    follower_pos=pos_follower,
                    follower_yaw=follower_yaw,  # ✓ Ora definito!
                    camera=camera,
                    leader_vel=leader_vel,
                    kp_distance=0.4,
                    kp_angle=0.3  # ← Ridotto per stabilità
                )
                
                print(f"      Velocity cmd: [{velocity_cmd[0]:7.2f}, {velocity_cmd[1]:7.2f}, {velocity_cmd[2]:7.2f}]")
                print(f"      Yaw cmd: {yaw_cmd:7.2f} rad/s ({np.degrees(yaw_cmd):7.1f}°/s)")
                
                # ✓ CORRETTO: Passa yaw_cmd alla send_command
                tracker.send_command(
                    velocity_cmd=velocity_cmd,
                    yaw_cmd=yaw_cmd,  # ✓ Parametro nominato corretto!
                    max_velocity=vmax
                )
            
            else:
                out_fov_count += 1
                if loop_count % 10 == 0:
                    print(f"[{loop_count:03d}] ✗ Leader NOT in FOV - Distance: {distance:6.2f}m")
            
            # ===== STEP 5: Rate limiting =====
            next_t += PERIOD
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                next_t = time.monotonic()
            
            loop_count += 1
    
    except KeyboardInterrupt:
        print("\n[*] Interrupted by user")
    
    except Exception as e:
        print(f"\n[!] Unexpected error: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        print("\n[*] Cleaning up...")
        print(f"    Stats: {in_fov_count} cycles IN FOV, {out_fov_count} cycles OUT of FOV")
        try:
            maneuver_l.wait()
        except:
            pass
        stop()
        print("[+] Done")


if __name__ == "__main__":
    main()