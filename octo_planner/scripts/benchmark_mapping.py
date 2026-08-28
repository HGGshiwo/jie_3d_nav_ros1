#!/usr/bin/env python3
import rospy
import time
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap

class MappingBenchmark:
    def __init__(self, pc_topic, octomap_topic):
        self.pc_topic = pc_topic
        self.octomap_topic = octomap_topic
        
        self.pc_times = {}
        self.latencies = []
        self.octomap_arrival_times = []
        self.pc_received_count = 0
        
        rospy.loginfo(f"Subscribing to PointCloud: {pc_topic}")
        rospy.loginfo(f"Subscribing to Octomap: {octomap_topic}")
        
        self.pc_sub = rospy.Subscriber(pc_topic, PointCloud2, self.pc_callback, queue_size=10)
        self.octomap_sub = rospy.Subscriber(octomap_topic, Octomap, self.octomap_callback, queue_size=10)
        
    def pc_callback(self, msg):
        # Store wall-clock time using high precision perf_counter
        stamp_ns = msg.header.stamp.to_nsec()
        self.pc_times[stamp_ns] = time.perf_counter()
        self.pc_received_count += 1
        rospy.loginfo(f"Received PointCloud #{self.pc_received_count} | stamp: {msg.header.stamp.to_sec():.4f}")
        
        # Keep dictionary size bounded
        if len(self.pc_times) > 200:
            # remove oldest
            oldest_key = min(self.pc_times.keys())
            del self.pc_times[oldest_key]
            
    def octomap_callback(self, msg):
        arrival_time = time.perf_counter()
        self.octomap_arrival_times.append(arrival_time)
        rospy.loginfo(f"Received Octomap     #{len(self.octomap_arrival_times)} | stamp: {msg.header.stamp.to_sec():.4f}")
        
        stamp_ns = msg.header.stamp.to_nsec()
        matched = False
        latency = 0.0
        
        # 1. Try exact stamp match
        if stamp_ns in self.pc_times:
            latency = (arrival_time - self.pc_times[stamp_ns]) * 1000.0  # ms
            self.latencies.append(latency)
            del self.pc_times[stamp_ns]
            matched = True
        else:
            # 2. Try approximate stamp match (within 1.0s difference)
            msg_sec = msg.header.stamp.to_sec()
            best_ns = None
            best_diff = 1.0  # max 1.0s difference
            for pc_ns in list(self.pc_times.keys()):
                pc_sec = pc_ns / 1e9
                diff = abs(msg_sec - pc_sec)
                if diff < best_diff:
                    best_diff = diff
                    best_ns = pc_ns
            if best_ns is not None:
                latency = (arrival_time - self.pc_times[best_ns]) * 1000.0
                self.latencies.append(latency)
                del self.pc_times[best_ns]
                matched = True
                
        if matched:
            rospy.loginfo(f"--> MATCHED Frame #{len(self.latencies)} | Latency: {latency:.2f} ms")
                
    def print_results(self):
        print("\n" + "="*50)
        print("           MAPPING PERFORMANCE BENCHMARK            ")
        print("="*50)
        print(f"PointCloud messages received: {self.pc_received_count} (topic: {self.pc_topic})")
        print(f"Octomap messages received:    {len(self.octomap_arrival_times)} (topic: {self.octomap_topic})")
        print("-"*50)
        
        if not self.latencies:
            print("No matching frames processed.")
            print("Please ensure:")
            print("1. Both topics are publishing active data.")
            print("2. The correct topic names are passed via parameters.")
            print("="*50)
            return
            
        avg_latency = sum(self.latencies) / len(self.latencies)
        max_latency = max(self.latencies)
        min_latency = min(self.latencies)
        
        # Calculate frequency
        hz = 0.0
        if len(self.octomap_arrival_times) > 1:
            duration = self.octomap_arrival_times[-1] - self.octomap_arrival_times[0]
            if duration > 0:
                hz = (len(self.octomap_arrival_times) - 1) / duration
                
        print(f"Total Matched Frames:  {len(self.latencies)}")
        print(f"Publish Frequency:     {hz:.2f} Hz")
        print(f"Average Latency:       {avg_latency:.2f} ms")
        print(f"Min Latency:           {min_latency:.2f} ms")
        print(f"Max Latency:           {max_latency:.2f} ms")
        print("="*50 + "\n")

if __name__ == '__main__':
    rospy.init_node('benchmark_mapping', anonymous=True)
    
    # Read parameters from launch or private namespace
    pc_topic = rospy.get_param('~pc_topic', '/lidar_points')
    octomap_topic = rospy.get_param('~octomap_topic', '/octomap_local')
    
    benchmark = MappingBenchmark(pc_topic, octomap_topic)
    
    rospy.on_shutdown(benchmark.print_results)
    rospy.spin()
