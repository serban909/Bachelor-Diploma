from pybricks.tools import wait

class PathLogger:
    """
    Logs robot decisions and actions.
    Can be used by any behavior that needs to track its path.
    """
    def __init__(self):
        self.log_entries = []
        self.decision_count = 0
        self.total_distance = 0
        
    def log_decision(self, decision_type, detail=""):
        """Log a decision point"""
        self.decision_count += 1
        entry = {
            'type': 'decision',
            'number': self.decision_count,
            'decision': decision_type,
            'detail': detail
        }
        self.log_entries.append(entry)
        self._print_entry(entry)
        
    def log_turn(self, angle, description=""):
        """Log a turn action"""
        entry = {
            'type': 'turn',
            'angle': angle,
            'description': description
        }
        self.log_entries.append(entry)
        self._print_entry(entry)
        
    def log_move(self, distance_mm):
        """Log a forward movement"""
        self.total_distance += distance_mm
        entry = {
            'type': 'move',
            'distance': distance_mm,
            'total_distance': self.total_distance
        }
        self.log_entries.append(entry)
        self._print_entry(entry)
        
    def log_sensor_reading(self, sensor_type, value, interpretation=""):
        """Log a sensor reading"""
        entry = {
            'type': 'sensor',
            'sensor': sensor_type,
            'value': value,
            'interpretation': interpretation
        }
        self.log_entries.append(entry)
        self._print_entry(entry)
        
    def _print_entry(self, entry):
        """Print log entry to console"""
        if entry['type'] == 'decision':
            msg = str(entry['number']) + ". DECISION: " + entry['decision']
            if entry['detail']:
                msg += " (" + entry['detail'] + ")"
            print(msg)
        elif entry['type'] == 'turn':
            angle_str = str(entry['angle'])
            print("  TURN: " + angle_str + " degrees - " + entry['description'])
        elif entry['type'] == 'move':
            print("  MOVE: " + str(entry['distance']) + "mm (Total: " + str(entry['total_distance']) + "mm)")
        elif entry['type'] == 'sensor':
            msg = "  SENSOR [" + entry['sensor'] + "]: " + str(entry['value'])
            if entry['interpretation']:
                msg += " -> " + entry['interpretation']
            print(msg)
    
    def get_summary(self):
        """Get summary statistics"""
        return {
            'total_decisions': self.decision_count,
            'total_distance': self.total_distance,
            'total_actions': len(self.log_entries)
        }
    
    def get_log_entries(self):
        """Get all log entries"""
        return self.log_entries
    
    def clear(self):
        """Clear all logs"""
        self.log_entries = []
        self.decision_count = 0
        self.total_distance = 0
