# =============================================================================
# config.py — Paramètres matériels et algorithmiques — CoVAPSy 2026
# =============================================================================
# COMMENT UTILISER CE FICHIER :
# Ce fichier est le seul à modifier le jour des essais.
# Chaque paramètre indique : valeur actuelle | plage valide | effet si on change.
# =============================================================================

# -----------------------------------------------------------------------------
# PWM — Général
# -----------------------------------------------------------------------------

PWM_HZ = 50                  # Fréquence PWM en Hz. Ne pas modifier (standard servo/ESC).

PWM_PROP_CHANNEL = 0         # Canal PWM propulsion — GPIO 12. Ne pas modifier.
PWM_DIR_CHANNEL  = 1         # Canal PWM direction  — GPIO 13. Ne pas modifier.

DUTY_MIN_CLAMP = 5.0         # Limite basse absolue de tout duty cycle envoyé.
DUTY_MAX_CLAMP = 10.0        # Limite haute absolue. Protège servo et ESC.

# -----------------------------------------------------------------------------
# Direction (servo) — valeurs calibrées
# -----------------------------------------------------------------------------

SERVO_DUTY_CENTER = 7.85    # Duty cycle (%) roues droites. Calibré terrain : 7.85.
                             # Si la voiture tire à gauche/droite en ligne droite :
                             # ajuster de ±0.05 jusqu'à rouler droit.

SERVO_DUTY_MIN = 6.85       # Duty cycle (%) butée gauche. Calibré terrain : 6.85.
                             # Ne pas descendre sous 6.00 (butée mécanique).

SERVO_DUTY_MAX = 8.60       # Duty cycle (%) butée droite. Calibré terrain : 8.60.
                             # Ne pas dépasser 10.00 (butée mécanique).

STEER_ANGLE_MAX_DEG = 18.0   # Débattement max en degrés de chaque côté.
                             # Correspond à SERVO_DUTY_MIN et SERVO_DUTY_MAX.
                             # Ne pas modifier sans re-calibrer les duty cycles.

# -----------------------------------------------------------------------------
# Propulsion (ESC) — valeurs calibrées
# -----------------------------------------------------------------------------

ESC_DUTY_NEUTRAL   = 7.78   # Point mort ESC. Calibré terrain : 7.78.
                             # Si le moteur tourne au repos : ajuster de ±0.05.
                             # ATTENTION : sur cet ESC, l'avant est EN DESSOUS du neutral.

ESC_DUTY_FWD_START = 7.88   # Seuil minimal de marche avant. Calibré terrain : 7.88.
                             # FWD_START > NEUTRAL : avancer = augmenter le duty au-dessus du neutral.
                             # En dessous de FWD_START : moteur à l'arrêt (zone morte).

# -----------------------------------------------------------------------------
# Reverse — NON UTILISÉ (mode FBR non fonctionnel sur cet ESC en configuration actuelle)
# -----------------------------------------------------------------------------
# La gestion de collision se fait par arrêt + virage, sans recul moteur.
# Ces paramètres sont conservés pour référence mais ignorés par le code.

ESC_DUTY_REV_START  = 7.20  # Duty pour engager le reverse.
ESC_DUTY_REV_STABLE = 6.70  # Duty pour reculer stable.
T_REVERSE_S = 1.0            # Durée du recul en secondes.
REVERSE_ENGAGE_S = 0.15      # Temps entre chaque étape de la séquence reverse (s).

# -----------------------------------------------------------------------------
# Lidar — RPLidar A2M12
# -----------------------------------------------------------------------------

PORT    = "/dev/ttyUSB0"     # Port série du lidar. Ne pas modifier.
BAUDRATE = 256000            # Vitesse série A2M12. Ne pas modifier (115200 pour A2M8).

LIDAR_FRESH_MAX_S = 0.50     # Scan ignoré si plus vieux que cette durée (secondes).
                             # Augmenter si beaucoup de scans sont rejetés.

LIDAR_RATE_WIN_S  = 2.0      # Fenêtre de calcul du taux de scan en secondes.
                             # Ne pas modifier.

# -----------------------------------------------------------------------------
# FTG — Follow The Gap
# -----------------------------------------------------------------------------

D_MIN_MM = 1500              # Distance de sécurité (mm). Tout point < D_MIN_MM
                             # est considéré comme obstacle et ferme le gap.
                             # Plage : 200–600.
                             # Augmenter → plus prudent, gaps plus petits.
                             # Diminuer → prend plus de risques, gaps plus grands.

W_MIN_DEG = 20               # Largeur angulaire minimale d'un gap valide (degrés).
                             # Plage : 10–40.
                             # Augmenter → ignore les petits gaps (plus sûr).
                             # Diminuer → accepte des passages étroits.

K_FTG = 1.0                  # Gain angulaire FTG : angle_commande = K_FTG * angle_gap_centre.
                             # Plage : 0.2–1.0.
                             # Augmenter → braquage plus agressif dans les virages.
                             # Diminuer → braquage plus doux, risque de rater les virages.

FTG_SECTOR_DEG = 150         # Demi-angle de la zone avant analysée par FTG (degrés).
                             # Plage : 60–120. Réduire si la voiture réagit à des
                             # obstacles latéraux non pertinents.

# -----------------------------------------------------------------------------
# Collision — seuil d'urgence
# -----------------------------------------------------------------------------

COLLISION_DIST_MM = 700      # Distance minimale avant ralentissement d'urgence (mm).
                             # Plage : 150–350.
                             # Mesurée sur le secteur avant ±COLLISION_SECTOR_DEG.
                             # Diminuer → réagit plus tard (plus rapide mais risqué).
                             # Augmenter → réagit tôt (plus sûr mais peut freiner inutilement).

COLLISION_SECTOR_DEG = 15    # Demi-angle du secteur de détection collision (degrés).
                             # Plage : 20–45. Ne pas modifier sans tester.

# -----------------------------------------------------------------------------
# Boucle de contrôle
# -----------------------------------------------------------------------------

CONTROL_HZ = 50              # Fréquence de la boucle de conduite (Hz).
                             # Ne pas modifier.

DT_S = 1.0 / CONTROL_HZ     # Période de la boucle en secondes. Calculé automatiquement.

# -----------------------------------------------------------------------------
# Vitesse
# -----------------------------------------------------------------------------

VITESSE_MS = 0.4             # Vitesse de croisière (m/s). Plage : 0.3–1.5.
                             # Commencer à 0.3 le jour des essais, augmenter si stable.

VITESSE_MIN = 0.28           # Vitesse plancher (m/s). Ne jamais descendre en dessous.
                             # En dessous de ~0.25 le moteur n'a plus de couple.

VITESSE_MAX_MS = 2.0         # Limite logicielle absolue. Ne pas dépasser.

# -----------------------------------------------------------------------------
# Détection blocage et recul
# -----------------------------------------------------------------------------

STUCK_DIST_MM = 400          # Distance frontale (mm) en dessous de laquelle on
                             # incrémente le compteur de blocage.
                             # Plage : 200–600.

STUCK_TICKS = 25             # Nombre de ticks consécutifs avant de déclencher
                             # la manœuvre de recul. À 50 Hz, 25 = 0.5 s.
                             # Plage : 15–50.

STUCK_AV_ZERO_TICKS = 15    # Nombre de ticks consécutifs avec av=0 (pas de
                             # mesure devant) ET murs latéraux proches → bloqué.
                             # À 50 Hz, 15 = 0.3 s. Plage : 10–30.
