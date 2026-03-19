# Module de commande des deux PWM hardware du Raspberry Pi 4.
# Expose la classe Actuators : direction (servo), propulsion (ESC), recul, arrêt.

from rpi_hardware_pwm import HardwarePWM
import time
import config
from steering import angle_deg_to_duty, clamp


class Actuators:

    def __init__(self):
        # Canal 0 = propulsion (ESC) sur GPIO 12.
        # Canal 1 = direction (servo) sur GPIO 13.
        self.pwm_prop = HardwarePWM(pwm_channel=config.PWM_PROP_CHANNEL, hz=config.PWM_HZ)
        self.pwm_dir  = HardwarePWM(pwm_channel=config.PWM_DIR_CHANNEL,  hz=config.PWM_HZ)
        # Démarrage au neutral (ESC s'arme) et servo centré (roues droites).
        self.pwm_prop.start(config.ESC_DUTY_NEUTRAL)
        self.pwm_dir.start(config.SERVO_DUTY_CENTER)

    def set_direction(self, angle_deg: float):
        # Inversion physique : le servo est câblé en sens inverse de la convention FTG.
        # FTG : +angle = droite, -angle = gauche.
        # Servo physique : +angle → tourne à gauche → on inverse.
        dc = angle_deg_to_duty(-angle_deg)
        self.pwm_dir.change_duty_cycle(dc)

    def set_vitesse(self, ms: float):
        # Problème de deadband : entre ESC_DUTY_NEUTRAL (7.78) et
        # ESC_DUTY_FWD_START (7.88) le moteur ne tourne pas (zone morte = 0.10 %).
        # On saute cette zone : ratio=0 → neutral (arrêt), ratio>0 → on part
        # de FWD_START et on monte linéairement jusqu'à DUTY_MAX_CLAMP (10.0).
        #
        # Formule : dc = NEUTRAL + dead + ratio × delta_max
        #   dead      = FWD_START - NEUTRAL = 7.88 - 7.78 = 0.10
        #   delta_max = DUTY_MAX_CLAMP - FWD_START = 10.0 - 7.88 = 1.12
        #
        # Exemples :
        #   ms=0.0 → ratio=0.00 → dc = 7.78 (arrêt)
        #   ms=0.5 → ratio=0.25 → dc = 7.78 + 0.10 + 0.25×1.12 = 8.16
        #   ms=2.0 → ratio=1.00 → dc = 7.78 + 0.10 + 1.00×1.12 = 9.00
        ratio = clamp(ms / config.VITESSE_MAX_MS, 0.0, 1.0)
        if ratio == 0:
            dc = config.ESC_DUTY_NEUTRAL
        else:
            dead = config.ESC_DUTY_FWD_START - config.ESC_DUTY_NEUTRAL
            delta_max = config.DUTY_MAX_CLAMP - config.ESC_DUTY_FWD_START
            dc = config.ESC_DUTY_NEUTRAL + dead + ratio * delta_max
        self.pwm_prop.change_duty_cycle(clamp(dc, config.DUTY_MIN_CLAMP, config.DUTY_MAX_CLAMP))

    def reculer(self, duree_s: float = None):
        # duree_s par défaut = config.T_REVERSE_S (0.5 s).
        if duree_s is None:
            duree_s = config.T_REVERSE_S
        # Séquence double-tap pour ESC hobby :
        #   1. Frein (en dessous du neutral) → l'ESC interprète comme "brake"
        #   2. Retour au neutral
        #   3. En dessous du neutral à nouveau → l'ESC engage le reverse
        # Étape 1 : frein
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_REV_START)
        time.sleep(config.REVERSE_ENGAGE_S)
        # Étape 2 : retour neutral
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_NEUTRAL)
        time.sleep(config.REVERSE_ENGAGE_S)
        # Étape 3 : reverse engagé
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_REV_START)
        time.sleep(config.REVERSE_ENGAGE_S)
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_REV_STABLE)
        time.sleep(duree_s)
        # Retour neutral
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_NEUTRAL)

    def stop(self):
        # Ordre : 1) recentrer le servo (évite de garder un braquage résiduel),
        # 2) couper la propulsion (neutral), 3) arrêter les signaux PWM.
        self.set_direction(0)
        self.pwm_prop.change_duty_cycle(config.ESC_DUTY_NEUTRAL)
        self.pwm_dir.stop()
        self.pwm_prop.stop()
