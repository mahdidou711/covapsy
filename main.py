# ~/covapsy/main.py
# Boucle de conduite principale — CoVAPSy 2026.
# Orchestre lidar, actuators et algorithme FTG.
#
# Machine à états :
#   AVANCE : conduite normale, FTG calcule l'angle, vitesse de croisière.
#   EVITE  : arrêt + virage vers le côté le plus dégagé après collision.
#
# Ordre de démarrage :
#   1. Lidar360 (thread d'acquisition)
#   2. LidarConsumer (lecture non-bloquante)
#   3. Actuators (PWM servo + ESC)
#   4. Attente 2 s (lidar prêt)
#   5. Boucle principale à CONTROL_HZ

import time
import signal
import sys
import config
from lidar_thread import Lidar360
from lidar_consumer import LidarConsumer
from actuators import Actuators
from ftg import compute_ftg, detect_collision


def main():
    # ── Création et démarrage des composants ──────────────────────────
    lidar    = Lidar360(config.PORT, config.BAUDRATE)
    consumer = LidarConsumer(lidar)
    act      = Actuators()
    lidar.start()

    # ── Gestionnaire de signal propre ─────────────────────────────────
    # SIGINT  = Ctrl+C depuis le terminal.
    # SIGTERM = kill depuis un autre process (ex: systemd, script de supervision).
    # Les deux doivent couper proprement les PWM et le lidar pour éviter
    # que le moteur reste alimenté ou que le servo reste braqué.
    def shutdown(signum, frame):
        act.stop()
        lidar.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    # ── try/finally global ─────────────────────────────────────────────
    # Garantit l'arrêt propre des actuateurs et du lidar quoi qu'il arrive
    # (Ctrl+C, signal, exception). Le signal handler reste en place pour
    # un arrêt immédiat, le finally couvre les cas restants.
    try:
        # ── Attente démarrage lidar ─────────────────────────────────────
        # Le lidar met ~1 s à démarrer le moteur et produire des scans valides.
        # On attend 2 s par sécurité pour ne pas démarrer la conduite sur des
        # scans vides ou incomplets.
        time.sleep(2)
        print("Lidar démarré, début de la conduite.")

        # ── Boucle principale ───────────────────────────────────────────
        etat = "AVANCE"
        ticks_sans_scan = 0
        # Compteur de blocage : incrémenté quand la voiture semble coincée.
        stuck_count = 0
        # Après un recul, phase avant forcée avec braquage escape_angle.
        escape_ticks = 0
        escape_angle = 0.0
        # Après la phase avant, cooldown pour ne pas re-déclencher stuck.
        escape_cooldown = 0

        while True:
            t0 = time.monotonic()

            # ── Fail-safe perte lidar ───────────────────────────────────
            # Lecture non-bloquante du dernier scan disponible.
            fresh, sid, age, scan, rate = consumer.poll()

            if not fresh:
                ticks_sans_scan += 1
                # Seuil atteint : couper propulsion et recentrer le servo.
                # Le warning n'est affiché qu'une seule fois (== 5, pas >= 5)
                # pour ne pas inonder le terminal.
                if ticks_sans_scan == 15:
                    act.set_vitesse(0)
                    act.set_direction(0)
                    print("[SECURITE] Lidar périmé, arrêt propulsion.")
            else:
                ticks_sans_scan = 0

            if etat == "AVANCE":

                if fresh and scan is not None:

                    # Distance frontale minimale (secteur ±COLLISION_SECTOR_DEG).
                    front_min = None
                    for i in list(range(0, config.COLLISION_SECTOR_DEG)) + \
                             list(range(360 - config.COLLISION_SECTOR_DEG, 360)):
                        d = scan[i]
                        if d > 0 and (front_min is None or d < front_min):
                            front_min = d

                    # Bug fix : si front_min=None (av=0, lidar ne mesure rien
                    # devant = trop proche), vérifier les murs latéraux.
                    # Si au moins un côté est < 1000 mm → on est probablement
                    # collé à un obstacle, forcer front_min à une valeur basse.
                    if front_min is None:
                        left_d = scan[90] if scan[90] > 0 else 100
                        right_d = scan[270] if scan[270] > 0 else 100
                        if left_d < 1000 or right_d < 1000:
                            front_min = 100  # Forcer "très proche"

                    # ── Phase avant forcée post-recul ─────────────────────
                    # Après le recul, on avance avec le braquage escape_angle
                    # pour s'éloigner du mur et partir du bon côté.
                    if escape_ticks > 0:
                        escape_ticks -= 1
                        print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | ESCAPE {escape_ticks} {escape_angle:+.0f}°")
                        act.set_direction(escape_angle)
                        act.set_vitesse(config.VITESSE_MIN)
                    else:
                        angle = compute_ftg(scan, config.D_MIN_MM, config.W_MIN_DEG, config.K_FTG,
                                            sector_deg=config.FTG_SECTOR_DEG,
                                            steer_limit_deg=config.STEER_ANGLE_MAX_DEG)

                        # Vitesse proportionnelle à la distance frontale.
                        if front_min is not None and front_min < config.D_MIN_MM:
                            ratio = max(0.0, (front_min - config.COLLISION_DIST_MM)) / \
                                    (config.D_MIN_MM - config.COLLISION_DIST_MM)
                            ratio = min(1.0, max(0.0, ratio))
                            vitesse = config.VITESSE_MIN + (config.VITESSE_MS - config.VITESSE_MIN) * ratio
                            print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | ftg={angle:+.1f}° v={vitesse:.2f}")
                        else:
                            vitesse = config.VITESSE_MS
                            print(f"av={scan[0]:4d} ga={scan[90]:4d} dr={scan[270]:4d} | ftg={angle:+.1f}°")

                        act.set_direction(angle)
                        act.set_vitesse(vitesse)

                    # ── Détection de blocage ──────────────────────────────
                    # Cooldown après recul pour ne pas re-déclencher immédiatement.
                    if escape_cooldown > 0:
                        escape_cooldown -= 1
                        stuck_count = 0
                    elif front_min is not None and front_min < config.STUCK_DIST_MM:
                        stuck_count += 1
                    else:
                        stuck_count = 0

                    if stuck_count >= config.STUCK_TICKS:
                        # Choisir le côté le plus dégagé pour le braquage EN RECULANT.
                        # IMPORTANT : 0 = lidar ne mesure pas = mur trop proche (~100mm),
                        # PAS espace libre. On traite 0 comme 100mm pour ne pas
                        # foncer vers un mur invisible.
                        left_d = scan[90] if scan[90] > 0 else 100
                        right_d = scan[270] if scan[270] > 0 else 100
                        if left_d > right_d:
                            escape_angle = -config.STEER_ANGLE_MAX_DEG  # gauche
                        else:
                            escape_angle = config.STEER_ANGLE_MAX_DEG   # droite

                        print(f"[RECUL] Bloqué {stuck_count} ticks → recul {config.T_REVERSE_S}s braquage {escape_angle:+.0f}°")
                        # Braquer AVANT de reculer → servo tourne pendant tout le recul.
                        # Mur gauche → escape_angle droite → recule à droite → repart à droite.
                        act.set_vitesse(0)
                        act.set_direction(escape_angle)
                        act.reculer(config.T_REVERSE_S)
                        # Phase avant forcée : 25 ticks (0.5s) avec même braquage
                        # pour continuer dans la direction dégagée.
                        escape_ticks = 25
                        escape_cooldown = 50
                        stuck_count = 0

            # Régulation de fréquence : on calcule le temps restant avant le
            # prochain tick pour maintenir CONTROL_HZ (50 Hz = 20 ms par tick).
            # Si le traitement a pris plus que DT_S, on ne dort pas et on
            # enchaîne immédiatement le tick suivant.
            dt_ecoule = time.monotonic() - t0
            dt_restant = config.DT_S - dt_ecoule
            if dt_restant > 0:
                time.sleep(dt_restant)

    finally:
        act.stop()
        lidar.stop()


if __name__ == "__main__":
    main()
