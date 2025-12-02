class DataExporter:
    """
    Exports robot data (logs, maps, etc.) to files.
    Formats data for potential ML training.
    """
    
    @staticmethod
    def export_log_to_file(logger, filename):
        """
        Export PathLogger data to a text file.
        Format suitable for ML training.
        """
        try:
            print("Attempting to write log to:", filename)
            with open(filename, 'w') as f:
                # Write header
                f.write("=== ROBOT PATH LOG ===\n")
                summary = logger.get_summary()
                f.write("Total Decisions: " + str(summary['total_decisions']) + "\n")
                f.write("Total Distance: " + str(summary['total_distance']) + "mm\n")
                f.write("Total Actions: " + str(summary['total_actions']) + "\n")
                f.write("\n=== LOG ENTRIES ===\n")
                
                # Write each log entry
                for entry in logger.get_log_entries():
                    if entry['type'] == 'decision':
                        line = "DECISION," + str(entry['number']) + "," + entry['decision']
                        if entry['detail']:
                            line += "," + entry['detail']
                        f.write(line + "\n")
                    elif entry['type'] == 'turn':
                        f.write("TURN," + str(entry['angle']) + "," + entry['description'] + "\n")
                    elif entry['type'] == 'move':
                        f.write("MOVE," + str(entry['distance']) + "," + str(entry['total_distance']) + "\n")
                    elif entry['type'] == 'sensor':
                        line = "SENSOR," + entry['sensor'] + "," + str(entry['value'])
                        if entry['interpretation']:
                            line += "," + entry['interpretation']
                        f.write(line + "\n")
                
            print("SUCCESS: Log exported to:", filename)
            return True
        except Exception as e:
            print("ERROR exporting log:", str(e))
            return False
    
    @staticmethod
    def export_combined_data(logger, base_filename):
        """
        Export log with related filename.
        Returns log_success
        """
        log_file = base_filename + "_log.txt"
        
        log_success = DataExporter.export_log_to_file(logger, log_file)
        
        return log_success
    
    @staticmethod
    def export_training_data(logger, filename):
        """
        Export data in ML-friendly format (CSV-like).
        Each line: decision_num,action_type,value1,value2,description
        """
        try:
            print("Attempting to write training data to:", filename)
            with open(filename, 'w') as f:
                # Write CSV header
                f.write("decision_num,action_type,value1,value2,description\n")
                
                current_decision = 0
                for entry in logger.get_log_entries():
                    if entry['type'] == 'decision':
                        current_decision = entry['number']
                        f.write(str(current_decision) + ",decision," + entry['decision'] + ",,\n")
                    elif entry['type'] == 'turn':
                        f.write(str(current_decision) + ",turn," + str(entry['angle']) + ",," + entry['description'] + "\n")
                    elif entry['type'] == 'move':
                        f.write(str(current_decision) + ",move," + str(entry['distance']) + "," + str(entry['total_distance']) + ",\n")
                    elif entry['type'] == 'sensor':
                        f.write(str(current_decision) + ",sensor_" + entry['sensor'] + "," + str(entry['value']) + ",," + entry['interpretation'] + "\n")
                
            print("SUCCESS: Training data exported to:", filename)
            return True
        except Exception as e:
            print("ERROR exporting training data:", str(e))
            return False
