#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import random
import math
import tf2_ros
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid
import actionlib
from actionlib_msgs.msg import GoalStatusArray  # 이 줄 추가
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from visualization_msgs.msg import Marker
import tf.transformations

class AutonomousExplorer:
    def __init__(self):
        rospy.init_node('autonomous_explorer', anonymous=True)
        
        # 상태 변수들 초기화
        self.map_data = None
        self.exploration_enabled = False
        self.is_moving = False  # 여기로 이동
        
        # 맵 데이터 구독
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)

        # autonomous 모드 활성화 여부 구독
        self.exploration_sub = rospy.Subscriber('/exploration_enable', Bool, self.exploration_callback)
        
        # 2D Nav Goal 퍼블리셔
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1, latch=False)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

        # move_base 상태 구독 추가
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        rospy.Timer(rospy.Duration(3.0), self.publish_random_goal)

    def map_callback(self, msg):
        self.map_data = msg

    def exploration_callback(self, msg):
        self.exploration_enabled = msg.data

    def status_callback(self, msg):
        """move_base의 상태를 확인하여 로봇이 이동 중인지 체크"""
        if len(msg.status_list) > 0:
            # status가 1이면 ACTIVE (실행 중)
            self.is_moving = msg.status_list[-1].status == 1

    def publish_random_goal(self, event):
        # rospy.loginfo("publish_random_goal 호출: exploration_enabled: %s, map_data: %s, is_moving: %s",
        #               self.exploration_enabled, self.map_data is not None, self.is_moving)
        
        # 탐색 비활성 또는 map 데이터가 없거나 현재 이동 중이면 목표 발행하지 않음
        if not self.exploration_enabled or self.map_data is None or self.is_moving:
        # if not self.exploration_enabled or self.map_data is None:
            # rospy.loginfo("publish_random_goal 조건 미충족, goal 발행 안함")
            return

        # 현재 로봇 위치 추정 (base_link -> map)
        try:
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            # 로봇의 현재 방향(yaw) 계산
            q = trans.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            self.publish_selection_region_marker()
        except Exception:
            rospy.logwarn("TF 변환 실패")
            return

        # 무작위 위치 생성 및 장애물 체크
        for _ in range(10):  # 최대 10회 시도
            dist = random.uniform(2.0, 6.0)
            # 로봇의 현재 방향을 기준으로 -90도~+90도 범위에서 랜덤 각도 생성
            relative_theta = random.uniform(-0.5 * math.pi, 0.5 * math.pi)
            theta = yaw + relative_theta  # 로봇 방향 + 상대 각도
            
            goal_x = trans.transform.translation.x + dist * math.cos(theta)
            goal_y = trans.transform.translation.y + dist * math.sin(theta)
            
            # 목표 지점 근처 0.02m 반경 내 빈 공간 확인
            if self.is_location_free(goal_x, goal_y):
                # goal 메시지 설정
                goal_msg = PoseStamped()
                goal_msg.header.frame_id = "map"
                goal_msg.header.stamp = rospy.Time.now()
                goal_msg.pose.position.x = goal_x
                goal_msg.pose.position.y = goal_y

                # goal 방향을 목표점을 향하도록 설정
                goal_orientation = math.atan2(goal_y - trans.transform.translation.y,
                                              goal_x - trans.transform.translation.x)
                q = tf.transformations.quaternion_from_euler(0, 0, goal_orientation)

                goal_msg.pose.orientation.x = q[0]
                goal_msg.pose.orientation.y = q[1]
                goal_msg.pose.orientation.z = q[2]
                goal_msg.pose.orientation.w = q[3]

                # 퍼블리시
                self.goal_pub.publish(goal_msg)
                rospy.loginfo("발행된 무작위 목표 지점: (%.2f, %.2f)" % (goal_x, goal_y))

                # MoveBaseGoal 설정 및 전송
                move_base_goal = MoveBaseGoal()
                move_base_goal.target_pose.header.frame_id = "map"
                move_base_goal.target_pose.pose.position.x = goal_x
                move_base_goal.target_pose.pose.position.y = goal_y
                move_base_goal.target_pose.pose.orientation = Quaternion(q[0], q[1], q[2], q[3])
                self.client.send_goal(move_base_goal, done_cb=self.goal_done_cb)
                return

        rospy.logwarn("장애물을 피한 무작위 위치를 찾지 못했습니다.")

    def is_location_free(self, x, y):
        """
        OccupancyGrid를 이용해 (x, y) 셀과 인접 8칸(3x3 grid)이 모두 0인지 확인.
        """
        if not self.map_data:
            return False
        
        map_info = self.map_data.info
        resolution = map_info.resolution
        origin_x = map_info.origin.position.x
        origin_y = map_info.origin.position.y
        width = map_info.width
        height = map_info.height
        data = self.map_data.data

        col = int((x - origin_x) / resolution)
        row = int((y - origin_y) / resolution)

        # 3x3 grid 범위를 확인 (중심 셀과 인접 8칸)
        for r in range(row - 1, row + 2):
            for c in range(col - 1, col + 2):
                if not (0 <= c < width and 0 <= r < height):
                    return False
                idx = r * width + c
                if data[idx] != 0:
                    return False
        return True

    def goal_done_cb(self, state, result):
        rospy.loginfo("goal_done_cb 호출: 상태=%s", state)
        # 상태 코드: 3 = SUCCEEDED, 2 = ABORTED (예시)
        if state == 3:
            rospy.loginfo("골 도착, 다음 목표를 설정합니다.")
        elif state == 2:
            rospy.logwarn("골 ABORT, 다음 목표를 설정합니다.")
        else:
            rospy.logwarn("골 실패 혹은 취소 (state: {})입니다. 목표 재발행하지 않음".format(state))
            return
        self.publish_random_goal(None)

    def publish_selection_region_marker(self):
        try:
            # 현재 로봇 위치와 방향 가져오기
            trans = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            current_x = trans.transform.translation.x
            current_y = trans.transform.translation.y
            
            # 현재 로봇의 방향(yaw) 계산
            q = trans.transform.rotation
            current_yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "selection_region"
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.05
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            points = []
            # 내측 호: 현재 위치 기준으로 반지름 1.0m (-90도 ~ +90도)
            for angle in range(-90, 91, 5):  # 5도 간격
                rad = math.radians(angle) + current_yaw  # 로봇 방향 기준
                pt = Point()
                pt.x = current_x + 1.0 * math.cos(rad)
                pt.y = current_y + 1.0 * math.sin(rad)
                pt.z = 0.0
                points.append(pt)
            
            # 외측 호: 현재 위치 기준으로 반지름 6.0m (-90도 ~ +90도)
            outer_points = []
            for angle in range(-90, 91, 5):
                rad = math.radians(angle) + current_yaw  # 로봇 방향 기준
                pt = Point()
                pt.x = current_x + 6.0 * math.cos(rad)
                pt.y = current_y + 6.0 * math.sin(rad)
                pt.z = 0.0
                outer_points.append(pt)
            outer_points.reverse()

            # 두 호를 연결하여 닫힌 영역 생성
            points.extend(outer_points)
            points.append(points[0])  # 시작점으로 연결
            
            marker.points = points
            self.marker_pub.publish(marker)

        except Exception as e:
            rospy.logwarn("마커 발행 실패: %s", str(e))

if __name__ == '__main__':
    try:
        explorer = AutonomousExplorer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
