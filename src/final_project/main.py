#!/usr/bin/env python3
"""
Main script per demo FOV camera e tracking
Legge posizioni direttamente da OptiTrack.bodies (raw mocap data)
Tralascia FSM - solo verifica camera.contains() e tracker.compute_command()
"""

import time
import numpy as np
from functions import *
from camera import *


def main():
    


    """Main loop per demo FOV + tracking"""
    
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
    camera = FOVPyramid(half_angle_deg=45, max_range=10.0)
    tracker = TrackingController(
        follower=maneuver_f,
        kp=0.1,  # Guadagno controllo
        velocity_estimator = VelocityEstimator(window_size=10, alpha=0.9)
    )
    


   
    # Imposta limiti di velocità
    vmax = 10.0  # m/s
    maneuver_f.set_velocity_limit(vmax, 1)
    maneuver_l.set_velocity_limit(1.0, 1)
    
    print("[*] Starting main loop (period: 0.1s, 10Hz)...")
    print("=" * 80)
    print("Reading positions from OptiTrack.bodies (raw mocap data)")
    print("=" * 80)
    
    PERIOD = 0.1  # 10 Hz
    next_t = time.monotonic()
    loop_count = 0
    max_loops = 300  # 30 secondi max
    
    # Statistiche
    in_fov_count = 0
    out_fov_count = 0
    pom_l.log_state('//home/matteogiovanelli/tk3lab-ws/src/final_project/pom.log')
    try:
        while loop_count < max_loops:
            loop_start = time.monotonic()
            
            # ===== STEP 1: Leggi posizioni da OptiTrack.bodies =====
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
            pos_in_body_frame = pos_leader - pos_follower
            distance = np.linalg.norm(pos_in_body_frame)
            
            # ===== STEP 3: Controlla se leader è nel FOV =====
            is_in_fov = camera.contains(pos_in_body_frame)
            
            # ===== STEP 4: Se nel FOV, calcola e invia comando =====
            if is_in_fov:
                in_fov_count += 1
                print(f"\n[{loop_count:03d}] ✓ LEADER IN FOV")
                print(f"      Leader pos (mocap): [{pos_leader[0]:7.2f}, {pos_leader[1]:7.2f}, {pos_leader[2]:7.2f}]")
                print(f"      Follower pos (mocap): [{pos_follower[0]:7.2f}, {pos_follower[1]:7.2f}, {pos_follower[2]:7.2f}]")
                print(f"      Relative pos: [{pos_in_body_frame[0]:7.2f}, {pos_in_body_frame[1]:7.2f}, {pos_in_body_frame[2]:7.2f}]")
                print(f"      Distance: {distance:6.2f}m")
                
                # Aggiorna stima velocità del leader
                leader_vel = tracker.velocity_estimator.update(pos_leader)
                print(f"      Leader vel (est): [{leader_vel[0]:7.2f}, {leader_vel[1]:7.2f}, {leader_vel[2]:7.2f}]")
                
                # Calcola comando di velocità
                velocity_cmd = tracker.compute_command(
                    leader_pos=pos_leader,
                    follower_pos=pos_follower,
                    leader_vel=leader_vel
                )
                
                # Saturazione velocità
                cmd_magnitude = np.linalg.norm(velocity_cmd)
                if cmd_magnitude > vmax:
                    velocity_cmd = (velocity_cmd / cmd_magnitude) * vmax
                    print(f"      [SAT] Velocity limited to {vmax} m/s")
                
                print(f"      Velocity cmd: [{velocity_cmd[0]:7.2f}, {velocity_cmd[1]:7.2f}, {velocity_cmd[2]:7.2f}] (mag: {np.linalg.norm(velocity_cmd):6.2f})")
                
                # Invia comando al follower
                tracker.send_command(velocity_cmd)
            
            else:
                out_fov_count += 1
                if loop_count % 10 == 0:  # Stampa ogni 10 cicli per non spammare
                    print(f"[{loop_count:03d}] ✗ Leader NOT in FOV - Distance: {distance:6.2f}m")
                
                # Aggiorna stima velocità del leader (ORA LOGGA ANCHE QUANDO NON è in FOV)
                leader_vel = tracker.velocity_estimator.update(pos_leader)
                print(f"      Leader vel (est): [{leader_vel[0]:7.2f}, {leader_vel[1]:7.2f}, {leader_vel[2]:7.2f}]")
            
            # ===== STEP 5: Rate limiting =====
            next_t += PERIOD
            sleep = next_t - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)
            else:
                # Overrun: resync
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
    pom_l.log_stop()
    