"""Camera recovery requires retention and a physically correct live observation.
Pose agreement is necessary but images must also be audited; score cannot bypass the sensor check.
"""
from dataclasses import dataclass
import math
@dataclass
class RecoveryScorer:
    retained_seconds: float=0.
    def update(self, *, latched, position_error_m, rotation_error_rad, other_hand_clear,
               sensor_position_error_m, sensor_rotation_error_rad, image_valid, dt=.05):
        mechanical=latched and position_error_m<.005 and rotation_error_rad<math.radians(5) and other_hand_clear
        sensor_ok=sensor_position_error_m<1e-4 and sensor_rotation_error_rad<1e-3 and image_valid
        self.retained_seconds=self.retained_seconds+dt if mechanical and sensor_ok else 0.
        return {'score':float(self.retained_seconds>=2.),'mechanical':bool(mechanical),
                'sensor_ok':bool(sensor_ok),'retained_seconds':self.retained_seconds}
