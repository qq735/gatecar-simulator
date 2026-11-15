"""Editable starter: sensor-only PD steering, stops on line loss or stop input.
This simple example does not implement roundabouts, crossings or gate guidance.
"""
def reset(config):
    return {'last_error': 0.0}

def step(sensors, state):
    # camera_gray is a flat list of 4800 values, indexed by row*80+column.
    # Use frame_new to run your own vision processing once per camera frame.
    error = sensors['visual_error_deg']
    derivative = (error-state['last_error']) / sensors['dt_s']
    state['last_error'] = error
    enabled = bool(sensors['line_valid']) and not sensors['kill']
    return {
        'speed_m_s': 0.7 if enabled else 0.0,
        'steer_deg': max(-30, min(30, 1.7*error + 0.006*derivative)),
        'fan': 0.35,
        'enabled': enabled,
        'gate_state': 0,
        'fault': 6 if sensors['kill'] else 0 if enabled else 3,
    }
