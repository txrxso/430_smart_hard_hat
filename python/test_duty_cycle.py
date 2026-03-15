"""
Duty Cycle Analyzer for ESP32 Gateway Module
Logs transmission durations and computes duty cycle statistics.

Usage:
  python test_duty_cycle.py --port COM3 --duration 3600 --scenario heartbeat_only --nodes 1
  python test_duty_cycle.py --port COM3 --duration 7200 --scenario alerts_30s --nodes 3
"""

import serial
import time
import argparse
import csv
from datetime import datetime, timedelta
from collections import defaultdict
import json
import os


class DutyCycleAnalyzer:
    def __init__(self, port, baudrate=115200, output_dir="python/data/duty_cycle"):
        self.port = port
        self.baudrate = baudrate
        self.output_dir = output_dir
        self.events = []
        self.start_time = None
        self.end_time = None
        
        # Event type mapping
        self.event_names = {
            'A': 'Alert',
            'H': 'Heartbeat',
            'W': 'WiFi_Reconnect',
            'M': 'MQTT_Connect'
        }
        
        # Ensure output directory exists
        os.makedirs(output_dir, exist_ok=True)
    
    def run_test(self, duration_seconds, scenario, num_nodes):
        """
        Run duty cycle test and log data.
        
        Args:
            duration_seconds: How long to collect data
            scenario: Test scenario name (heartbeat_only, alerts_30s, etc.)
            num_nodes: Number of nodes in CAN network (1-3)
        """
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{self.output_dir}/duty_cycle_{scenario}_nodes{num_nodes}_{timestamp}.csv"
        
        print(f"[*] Starting duty cycle test")
        print(f"[*] Scenario: {scenario}")
        print(f"[*] Nodes: {num_nodes}")
        print(f"[*] Duration: {duration_seconds}s ({duration_seconds/60:.1f} minutes)")
        print(f"[*] Output: {filename}")
        print(f"[*] Connecting to {self.port} @ {self.baudrate}...")
        
        try:
            ser = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)  # Let Arduino reset
            
            # Wait for startup marker
            print("[*] Waiting for DUTY_CYCLE_START marker...")
            while True:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                if "DUTY_CYCLE_START" in line:
                    print("[*] Marker detected! Starting data collection...")
                    break
                if line:
                    print(f"[DEBUG] {line}")
            
            self.start_time = time.time()
            end_target = self.start_time + duration_seconds
            
            # Open CSV file for logging
            with open(filename, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['system_timestamp', 'event_type', 'esp32_timestamp_ms', 
                                'duration_us', 'success'])
                
                event_count = 0
                last_status = time.time()
                
                while time.time() < end_target:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if not line:
                        continue
                    
                    # Parse CSV format: event_type,esp32_timestamp_ms,duration_us,success
                    parts = line.split(',')
                    if len(parts) == 4 and parts[0] in self.event_names:
                        try:
                            event_type = parts[0]
                            esp32_ts = int(parts[1])
                            duration_us = int(parts[2])
                            success = int(parts[3])
                            
                            system_ts = datetime.now().isoformat()
                            
                            # Write to CSV
                            writer.writerow([system_ts, event_type, esp32_ts, 
                                           duration_us, success])
                            csvfile.flush()  # Ensure data is written
                            
                            # Store in memory for analysis
                            self.events.append({
                                'event_type': event_type,
                                'esp32_timestamp_ms': esp32_ts,
                                'duration_us': duration_us,
                                'success': bool(success)
                            })
                            
                            event_count += 1
                            
                        except (ValueError, IndexError) as e:
                            print(f"[WARN] Parse error: {line} - {e}")
                    else:
                        # Debug output from ESP32
                        if line and not line.startswith('['):
                            print(f"[ESP32] {line}")
                    
                    # Status update every 30 seconds
                    if time.time() - last_status > 30:
                        elapsed = time.time() - self.start_time
                        remaining = end_target - time.time()
                        print(f"[*] Progress: {elapsed:.0f}s elapsed, {remaining:.0f}s remaining, "
                              f"{event_count} events logged")
                        last_status = time.time()
            
            self.end_time = time.time()
            ser.close()
            
            print(f"\n[*] Data collection complete!")
            print(f"[*] Total events: {event_count}")
            print(f"[*] CSV saved to: {filename}")
            
            # Analyze and generate report
            self.analyze(filename, scenario, num_nodes)
            
        except serial.SerialException as e:
            print(f"[ERROR] Serial port error: {e}")
        except KeyboardInterrupt:
            print("\n[*] Test interrupted by user")
            self.end_time = time.time()
            if hasattr(self, 'events') and len(self.events) > 0:
                self.analyze(filename, scenario, num_nodes)
    
    def analyze(self, csv_filename, scenario, num_nodes):
        """Analyze logged data and generate report."""
        if not self.events:
            print("[WARN] No events to analyze")
            return
        
        # Calculate total test duration
        test_duration_s = (self.end_time - self.start_time) if self.end_time else 0
        
        # Aggregate statistics by event type
        stats_by_type = defaultdict(lambda: {
            'count': 0,
            'total_duration_us': 0,
            'success_count': 0,
            'success_duration_us': 0,
            'failed_count': 0,
            'failed_duration_us': 0,
            'durations': []
        })
        
        # Overall statistics
        total_tx_time_us = 0
        total_success_duration_us = 0
        total_failed_duration_us = 0
        total_success_count = 0
        total_failed_count = 0
        
        for event in self.events:
            et = event['event_type']
            duration = event['duration_us']
            success = event['success']
            
            # Per event type stats
            stats_by_type[et]['count'] += 1
            stats_by_type[et]['total_duration_us'] += duration
            stats_by_type[et]['durations'].append(duration)
            
            # Track success/failure per event type
            if success:
                stats_by_type[et]['success_count'] += 1
                stats_by_type[et]['success_duration_us'] += duration
            else:
                stats_by_type[et]['failed_count'] += 1
                stats_by_type[et]['failed_duration_us'] += duration
            
            # Overall totals
            total_tx_time_us += duration
            if success:
                total_success_duration_us += duration
                total_success_count += 1
            else:
                total_failed_duration_us += duration
                total_failed_count += 1
        
        # Calculate duty cycle
        test_duration_us = test_duration_s * 1_000_000
        duty_cycle_pct = (total_tx_time_us / test_duration_us * 100) if test_duration_us > 0 else 0
        success_duty_cycle_pct = (total_success_duration_us / test_duration_us * 100) if test_duration_us > 0 else 0
        failed_duty_cycle_pct = (total_failed_duration_us / test_duration_us * 100) if test_duration_us > 0 else 0
        
        # Generate report structure
        report = {
            'scenario': scenario,
            'num_nodes': num_nodes,
            'test_duration_s': test_duration_s,
            'total_events': len(self.events),
            'total_tx_time_us': total_tx_time_us,
            'total_tx_time_s': total_tx_time_us / 1_000_000,
            'duty_cycle_pct': duty_cycle_pct,
            'success_stats': {
                'count': total_success_count,
                'duration_us': total_success_duration_us,
                'duration_s': total_success_duration_us / 1_000_000,
                'duty_cycle_pct': success_duty_cycle_pct
            },
            'failed_stats': {
                'count': total_failed_count,
                'duration_us': total_failed_duration_us,
                'duration_s': total_failed_duration_us / 1_000_000,
                'duty_cycle_pct': failed_duty_cycle_pct
            },
            'events_by_type': {}
        }
        
        # Print report
        print("\n" + "="*80)
        print(f"{'DUTY CYCLE ANALYSIS REPORT':^80}")
        print("="*80)
        print(f"Scenario:          {scenario}")
        print(f"Nodes:             {num_nodes}")
        print(f"Test Duration:     {test_duration_s:.2f}s ({test_duration_s/60:.2f} min)")
        print(f"Total Events:      {len(self.events)}")
        print("-"*80)
        print(f"OVERALL DUTY CYCLE:     {duty_cycle_pct:.6f}%")
        print(f"Total TX Time:          {total_tx_time_us/1_000_000:.4f}s ({total_tx_time_us/1000:.2f}ms)")
        print("="*80)
        
        # Success vs Failed breakdown
        print("\nSUCCESS vs FAILED BREAKDOWN:")
        print("-"*80)
        print(f"{'Status':<15} {'Count':<10} {'Total Time (s)':<18} {'Duty Cycle %':<15}")
        print("-"*80)
        print(f"{'SUCCESS':<15} {total_success_count:<10} {total_success_duration_us/1_000_000:<18.4f} {success_duty_cycle_pct:<15.6f}")
        print(f"{'FAILED':<15} {total_failed_count:<10} {total_failed_duration_us/1_000_000:<18.4f} {failed_duty_cycle_pct:<15.6f}")
        print("-"*80)
        print(f"{'TOTAL':<15} {len(self.events):<10} {total_tx_time_us/1_000_000:<18.4f} {duty_cycle_pct:<15.6f}")
        print("="*80)
        
        # Breakdown by event type
        print("\nBREAKDOWN BY EVENT TYPE:")
        print("-"*80)
        print(f"{'Type':<15} {'Count':<8} {'Success':<8} {'Failed':<8} {'Total (ms)':<12} {'Avg (ms)':<12} {'Duty %':<10}")
        print("-"*80)
        
        for event_type in sorted(stats_by_type.keys()):
            s = stats_by_type[event_type]
            avg_duration_ms = (s['total_duration_us'] / s['count']) / 1000 if s['count'] > 0 else 0
            total_time_ms = s['total_duration_us'] / 1000
            event_duty_cycle = (s['total_duration_us'] / test_duration_us * 100) if test_duration_us > 0 else 0
            
            event_name = self.event_names.get(event_type, event_type)
            print(f"{event_name:<15} {s['count']:<8} {s['success_count']:<8} "
                  f"{s['failed_count']:<8} {total_time_ms:<12.2f} {avg_duration_ms:<12.2f} {event_duty_cycle:<10.6f}")
            
            # Add detailed stats to report
            report['events_by_type'][event_name] = {
                'count': s['count'],
                'success_count': s['success_count'],
                'success_duration_us': s['success_duration_us'],
                'failed_count': s['failed_count'],
                'failed_duration_us': s['failed_duration_us'],
                'total_duration_us': s['total_duration_us'],
                'avg_duration_ms': avg_duration_ms,
                'min_duration_ms': min(s['durations']) / 1000 if s['durations'] else 0,
                'max_duration_ms': max(s['durations']) / 1000 if s['durations'] else 0,
                'duty_cycle_pct': event_duty_cycle
            }
        
        print("-"*80)
        
        # Detailed per-event-type success/failure breakdown
        print("\nDETAILED SUCCESS/FAILURE BY EVENT TYPE:")
        print("-"*80)
        print(f"{'Type':<15} {'Status':<10} {'Count':<8} {'Duration (ms)':<15} {'% of Type':<12}")
        print("-"*80)
        
        for event_type in sorted(stats_by_type.keys()):
            s = stats_by_type[event_type]
            event_name = self.event_names.get(event_type, event_type)
            
            # Success row
            success_pct = (s['success_duration_us'] / s['total_duration_us'] * 100) if s['total_duration_us'] > 0 else 0
            print(f"{event_name:<15} {'SUCCESS':<10} {s['success_count']:<8} {s['success_duration_us']/1000:<15.2f} {success_pct:<12.2f}")
            
            # Failed row
            failed_pct = (s['failed_duration_us'] / s['total_duration_us'] * 100) if s['total_duration_us'] > 0 else 0
            print(f"{'':<15} {'FAILED':<10} {s['failed_count']:<8} {s['failed_duration_us']/1000:<15.2f} {failed_pct:<12.2f}")
            print("-"*80)
        
        print("\n" + "="*80)
        print("SUMMARY:")
        print("-"*80)
        print(f"Active (TX) Time:       {total_tx_time_us/1_000_000:.4f}s / {test_duration_s:.2f}s")
        print(f"  - Successful TX:      {total_success_duration_us/1_000_000:.4f}s ({success_duty_cycle_pct:.6f}%)")
        print(f"  - Failed TX:          {total_failed_duration_us/1_000_000:.4f}s ({failed_duty_cycle_pct:.6f}%)")
        print(f"Overall Duty Cycle:     {duty_cycle_pct:.6f}%")
        print(f"Idle Time:              {(100 - duty_cycle_pct):.6f}%")
        print("="*80 + "\n")
        
        # Save JSON report
        report_filename = csv_filename.replace('.csv', '_report.json')
        with open(report_filename, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"[*] JSON report saved to: {report_filename}")
        
        return report


def main():
    parser = argparse.ArgumentParser(description='ESP32 Duty Cycle Analyzer')
    parser.add_argument('--port', required=True, help='Serial port (e.g., COM3, /dev/ttyUSB0)')
    parser.add_argument('--duration', type=int, default=3600, 
                       help='Test duration in seconds (default: 3600 = 1 hour)')
    parser.add_argument('--scenario', required=True,
                       choices=['heartbeat_only', 'alerts_30s', 'alerts_5min', 
                               'alerts_10min', 'mixed', 'stress_test'],
                       help='Test scenario')
    parser.add_argument('--nodes', type=int, choices=[1, 2, 3], default=1,
                       help='Number of CAN nodes (1-3)')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Serial baudrate (default: 115200)')
    parser.add_argument('--output-dir', default='python/data',
                       help='Output directory for logs (default: python/data)')
    
    args = parser.parse_args()
    
    # Print test configuration
    print("\n" + "="*80)
    print(f"{'ESP32 GATEWAY DUTY CYCLE TEST':^80}")
    print("="*80)
    print(f"Port:       {args.port}")
    print(f"Duration:   {args.duration}s ({args.duration/60:.1f} min, {args.duration/3600:.2f} hr)")
    print(f"Scenario:   {args.scenario}")
    print(f"Nodes:      {args.nodes}")
    print("="*80 + "\n")
    
    analyzer = DutyCycleAnalyzer(args.port, args.baudrate, args.output_dir)
    analyzer.run_test(args.duration, args.scenario, args.nodes)


if __name__ == "__main__":
    main()

