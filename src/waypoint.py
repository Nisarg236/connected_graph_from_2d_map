from geometry_msgs.msg import PointStamped
import cv2
import numpy as np
import yaml
import time
import rospy
import math
import copy
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point
from tf.broadcaster import TransformBroadcaster
from visualization_msgs.msg import Marker
import os
from geometry_msgs.msg import Point
from random import random
from math import sin

global press_count
press_count = 1
server = None
menu_handler = MenuHandler()
br = None
counter = 0
global count
count = 0
global graph
graph = {}
global line_pub
line_pub = rospy.Publisher("edges", Marker, queue_size=10)
global lines
lines = {}
global make_edge_bw
make_edge_bw = []
global delete_edge_bw
delete_edge_bw = []


def frameCallback(msg):
    global counter, br
    time = rospy.Time.now()
    br.sendTransform(
        (0, 0, sin(counter / 140.0) * 2.0),
        (0, 0, 0, 1.0),
        time,
        "base_link",
        "moving_frame",
    )
    counter += 1


def processFeedback(feedback):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"

    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id
        way_pts[int(feedback.marker_name)] = [
            feedback.mouse_point.x,
            feedback.mouse_point.y,
        ]
        print(
            "UPDATED DICTIONARY FOR WAYPOINT :",
            feedback.marker_name,
            "  [x,y] = ",
            way_pts[int(feedback.marker_name)],
        )

    if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
        rospy.loginfo(s + ": button click" + mp + ".")

    elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
        if feedback.menu_entry_id == 1:
            server.erase(feedback.marker_name)
            print("DELETED WAYPOINT :", feedback.marker_name)
            del way_pts[int(feedback.marker_name)]
            print("UPDATED DICTIONARY")
            server.applyChanges()

        if feedback.menu_entry_id == 2:
            global im
            global line_pub
            yaml_string = yaml.dump(way_pts)
            print(yaml_string)

            for i in way_pts:
                x = way_pts[i][0]
                y = way_pts[i][1]
                cv2.circle(
                    im, (int(((x + 10) / 0.05)), int(((y + 10) / 0.05))), 10, 255, -1
                )

            im = 255 - im
            cv2.imwrite("broken_roads.png", im)
            im = cv2.imread("broken_roads.png")
            im_orig = im
            im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
            im = cv2.threshold(im, 100, 255, cv2.THRESH_BINARY)[1]
            _, contours, _ = cv2.findContours(
                im, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
            )
            global graph
            print("Number of edges found = ", len(contours))
            global indx
            indx = 0
            for contour in contours:
                extLeft = tuple(contour[contour[:, :, 0].argmin()][0])
                extRight = tuple(contour[contour[:, :, 0].argmax()][0])
                extTop = tuple(contour[contour[:, :, 1].argmin()][0])
                extBot = tuple(contour[contour[:, :, 1].argmax()][0])
                dist_e_w = (
                    (extLeft[0] - extRight[1]) ** 2 + (extLeft[0] - extRight[1]) ** 2
                ) ** 0.5
                dist_n_s = (
                    (extTop[0] - extBot[1]) ** 2 + (extTop[0] - extBot[1]) ** 2
                ) ** 0.5
                if dist_e_w > dist_n_s:
                    p1 = [extLeft[0], extLeft[1]]
                    p2 = [extRight[0], extRight[1]]
                else:
                    p1 = [extTop[0], extTop[1]]
                    p2 = [extBot[0], extBot[1]]

                cv2.circle(im_orig, (p1[0], p1[1]), 5, (0, 0, 255), -1)
                cv2.circle(im_orig, (p2[0], p2[1]), 5, (0, 0, 255), -1)

                p1_arr = np.array(p1)
                p2_arr = np.array(p2)

                p1_key = None
                p1_dist = float("inf")
                p2_key = None
                p2_dist = float("inf")
                for key, value in way_pts.items():
                    value_arr = np.array(value)
                    value_arr = (value_arr + 10) / 0.05
                    dist_p1 = np.linalg.norm(value_arr - p1_arr)
                    if dist_p1 < p1_dist:
                        p1_dist = dist_p1
                        p1_key = key
                    dist_p2 = np.linalg.norm(value_arr - p2_arr)
                    if dist_p2 < p2_dist:
                        p2_dist = dist_p2
                        p2_key = key

                p1_idx = p1_key
                p2_idx = p2_key

                lines[(p1_idx, p2_idx, indx)] = [
                    [way_pts[p1_idx][0], way_pts[p1_idx][1]],
                    [way_pts[p2_idx][0], way_pts[p2_idx][1]],
                ]

                line = Marker()
                line.header.frame_id = "map"
                line.header.stamp = rospy.Time.now()
                line.ns = "edges"
                line.id = indx
                indx += 1
                line.type = Marker.LINE_STRIP
                line.action = Marker.ADD
                line.scale.x = 0.3
                line.color.r = 0.0
                line.color.g = 1.0
                line.color.b = 0.0
                line.color.a = 1.0

                point1 = Point()
                point1.x = way_pts[p1_idx][0]
                point1.y = way_pts[p1_idx][1]
                point1.z = 0.0

                point2 = Point()
                point2.x = way_pts[p2_idx][0]
                point2.y = way_pts[p2_idx][1]
                point2.z = 0.0

                line.points.append(point1)
                line.points.append(point2)
                line_pub.publish(line)
                time.sleep(0.1)
                cv2.line(
                    im_orig,
                    (
                        int((way_pts[p1_idx][0] + 10) / 0.05),
                        int((way_pts[p1_idx][1] + 10) / 0.05),
                    ),
                    (
                        int((way_pts[p2_idx][0] + 10) / 0.05),
                        int((way_pts[p2_idx][1] + 10) / 0.05),
                    ),
                    (255, 0, 0),
                    1,
                )

                perimeter = cv2.arcLength(contour, True)
                perimeter = ((perimeter / 2) + 10) * 0.05

                if p1_idx not in graph:
                    graph[p1_idx] = {
                        "node_pos": way_pts[p1_idx],
                        "outward_edges": {p2_idx: perimeter},
                    }
                else:
                    graph[p1_idx]["outward_edges"][p2_idx] = perimeter

                if p2_idx not in graph:
                    graph[p2_idx] = {
                        "node_pos": way_pts[p2_idx],
                        "outward_edges": {p1_idx: perimeter},
                    }
                else:
                    graph[p2_idx]["outward_edges"][p1_idx] = perimeter

            cv2.imwrite("output_graph.png", im_orig)

            for node, node_data in graph.items():
                if node in node_data["outward_edges"]:
                    del node_data["outward_edges"][node]
                    print("removed")

            for i in range(0, 5):
                print("")
            print("DONE MAKING THE GRAPH")
            for i in range(0, 5):
                print("")

        if feedback.menu_entry_id == 3:
            global count
            global make_edge_bw
            make_edge_bw.append(int(feedback.marker_name))
            if len(make_edge_bw) > 2:
                make_edge_bw = [make_edge_bw[0], make_edge_bw[1]]

            print(make_edge_bw)
            print("")
            print("")
            print(make_edge_bw)
            print("")
            print("")
            if len(make_edge_bw) == 2:
                try:
                    count + 1
                    print("BOTH POINTS SELECTED")
                    eu_dist = (
                        (way_pts[make_edge_bw[0]][0] - way_pts[make_edge_bw[1]][0]) ** 2
                        + (way_pts[make_edge_bw[0]][1] - way_pts[make_edge_bw[1]][1])
                        ** 2
                    ) ** (0.5)

                    graph[make_edge_bw[0]]["outward_edges"][make_edge_bw[1]] = eu_dist
                    graph[make_edge_bw[1]]["outward_edges"][make_edge_bw[0]] = eu_dist
                    print(graph[make_edge_bw[0]])
                    print(graph[make_edge_bw[1]])

                    line = Marker()
                    line.header.frame_id = "map"
                    line.header.stamp = rospy.Time.now()
                    line.ns = "edges"

                    ids = []
                    for i in lines:
                        ids.append(int(i[2]))
                    id = max(ids) + 1
                    line.id = id
                    line.type = Marker.LINE_STRIP
                    line.action = Marker.ADD
                    line.scale.x = 0.3
                    line.color.r = 0.0
                    line.color.g = 1.0
                    line.color.b = 0.0
                    line.color.a = 1.0

                    point1 = Point()
                    point1.x = way_pts[make_edge_bw[0]][0]
                    point1.y = way_pts[make_edge_bw[0]][1]
                    point1.z = 0.0

                    point2 = Point()
                    point2.x = way_pts[make_edge_bw[1]][0]
                    point2.y = way_pts[make_edge_bw[1]][1]
                    point2.z = 0.0

                    line.points.append(point1)
                    line.points.append(point2)
                    line_pub.publish(line)
                    lines[(make_edge_bw[0], make_edge_bw[1], id)] = [
                        [way_pts[make_edge_bw[0]][0], way_pts[make_edge_bw[0]][1]],
                        [way_pts[make_edge_bw[1]][0], way_pts[make_edge_bw[1]][1]],
                    ]
                    make_edge_bw = []
                except Exception as e:
                    print(e)
                    make_edge_bw = []

        if feedback.menu_entry_id == 4:
            print("YOU PRESSED DELETE")
            global delete_edge_bw
            delete_edge_bw.append(int(feedback.marker_name))
            print(" ")
            print(delete_edge_bw)
            print("")
            if len(delete_edge_bw) == 2:
                print("BEFORE DELETE :")
                print(graph[delete_edge_bw[0]])
                print(graph[delete_edge_bw[1]])
                print("AFTER DELETE")
                del graph[delete_edge_bw[0]]["outward_edges"][delete_edge_bw[1]]
                del graph[delete_edge_bw[1]]["outward_edges"][delete_edge_bw[0]]
                print(graph[delete_edge_bw[0]])
                print(graph[delete_edge_bw[1]])

                print("NO OF LINES: ", len(lines))

                for i in lines.keys():
                    if [i[0], i[1]] == delete_edge_bw:
                        del lines[i]
                        line = Marker()
                        line.header.frame_id = "map"
                        line.ns = "edges"
                        line.id = i[2]
                        line.action = line.DELETE
                        line_pub.publish(line)
                        print("REMOVEDDDDDDDDDDDDDDDDDDDDDDDDDDDDd")
                for i in lines.keys():
                    if [i[1], i[0]] == delete_edge_bw:
                        del lines[i]
                        line = Marker()
                        line.header.frame_id = "map"
                        line.ns = "edges"
                        line.id = i[2]
                        line.action = line.DELETE
                        line_pub.publish(line)
                        print("REMOVED")

                delete_edge_bw = []

        if feedback.menu_entry_id == 5:
            if load == 0:
                global press_count
                print(press_count)

                if press_count == 1:
                    print("YOU PRESSED SAVE TO YAMl")

                    file = open(
                        home
                        + "/Downloads/waypoint/output_yaml/connected_graph_low_level.yaml",
                        "w",
                    )
                    yaml.dump(graph, file)
                    file.close()

                    file = open(
                        home
                        + "//Downloads/waypoint/output_yaml/way_pts_low_level.yaml",
                        "w",
                    )
                    yaml.dump(way_pts, file)
                    file.close()

                    file = open(
                        home + "/Downloads/waypoint/output_yaml/edges_low_level.yaml",
                        "w",
                    )
                    yaml.dump(lines, file)
                    print(lines)
                    file.close()
                    press_count += 1

                    line = Marker()
                    line.header.frame_id = "map"
                    line.ns = "edges"
                    line.action = line.DELETEALL
                    line_pub.publish(line)

                    img = gray
                    img = cv2.flip(img, 0)
                    img = cv2.threshold(img, 100, 255, cv2.THRESH_BINARY)[1]

                    corners = cv2.goodFeaturesToTrack(img, 1000, 0.375, 50)
                    corners = np.int0(corners)
                    for i in corners:
                        x, y = i.ravel()
                        cv2.circle(img, (x, y), 10, 0, -1)

                    _, contours, _ = cv2.findContours(
                        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
                    )

                    keypoints = []
                    global index
                    interval = 200
                    min_distance = 150
                    for i in list(way_pts.values()):
                        pt = [(i[0] + 10) / 0.05, (i[1] + 10) / 0.05]
                        keypoints.append(pt)
                    for contour in contours:
                        points = contour.reshape(-1, 2)
                        for i in range(0, len(points), interval):
                            keypoint = tuple(points[i])
                            if not any(
                                math.sqrt(
                                    (keypoint[0] - kp[0]) ** 2
                                    + (keypoint[1] - kp[1]) ** 2
                                )
                                < min_distance
                                for kp in keypoints
                            ):
                                keypoints.append(keypoint)
                                cv2.circle(
                                    img, (int(keypoint[0]), int(keypoint[1])), 10, 0, -1
                                )
                                way_pts[index] = [
                                    float(keypoint[0] * 0.05 - 10.0),
                                    float(keypoint[1] * 0.05 - 10.0),
                                ]
                                x = way_pts[index][0]
                                y = way_pts[index][1]
                                position = Point(x, y, 0)
                                make6DofMarker(
                                    False,
                                    InteractiveMarkerControl.NONE,
                                    position,
                                    True,
                                    name=str(index),
                                )
                                server.applyChanges()
                                index += 1

                    _, contours, _ = cv2.findContours(
                        img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE
                    )

                    for contour in contours:
                        extLeft = tuple(contour[contour[:, :, 0].argmin()][0])
                        extRight = tuple(contour[contour[:, :, 0].argmax()][0])
                        extTop = tuple(contour[contour[:, :, 1].argmin()][0])
                        extBot = tuple(contour[contour[:, :, 1].argmax()][0])
                        dist_e_w = (
                            (extLeft[0] - extRight[1]) ** 2
                            + (extLeft[0] - extRight[1]) ** 2
                        ) ** 0.5
                        dist_n_s = (
                            (extTop[0] - extBot[1]) ** 2 + (extTop[0] - extBot[1]) ** 2
                        ) ** 0.5
                        if dist_e_w > dist_n_s:
                            p1 = [extLeft[0], extLeft[1]]
                            p2 = [extRight[0], extRight[1]]
                        else:
                            p1 = [extTop[0], extTop[1]]
                            p2 = [extBot[0], extBot[1]]

                        way_pts_values = np.array(list(way_pts.values()))
                        p1_arr = np.array(p1)
                        p2_arr = np.array(p2)

                        p1_key = None
                        p1_dist = float("inf")
                        p2_key = None
                        p2_dist = float("inf")
                        for key, value in way_pts.items():
                            value_arr = np.array(value)
                            value_arr = (value_arr + 10) / 0.05
                            dist_p1 = np.linalg.norm(value_arr - p1_arr)
                            if dist_p1 < p1_dist:
                                p1_dist = dist_p1
                                p1_key = key
                            dist_p2 = np.linalg.norm(value_arr - p2_arr)
                            if dist_p2 < p2_dist:
                                p2_dist = dist_p2
                                p2_key = key

                        p1_idx = p1_key
                        p2_idx = p2_key

                        lines[(p1_idx, p2_idx, indx)] = [
                            [way_pts[p1_idx][0], way_pts[p1_idx][1]],
                            [way_pts[p2_idx][0], way_pts[p2_idx][1]],
                        ]
                        line = Marker()
                        line.header.frame_id = "map"
                        line.header.stamp = rospy.Time.now()
                        line.ns = "edges"
                        line.id = indx
                        indx += 1
                        line.type = Marker.LINE_STRIP
                        line.action = Marker.ADD
                        line.scale.x = 0.3
                        line.color.r = 0.0
                        line.color.g = 1.0
                        line.color.b = 0.0
                        line.color.a = 1.0

                        point1 = Point()
                        point1.x = way_pts[p1_idx][0]
                        point1.y = way_pts[p1_idx][1]
                        point1.z = 0.0

                        point2 = Point()
                        point2.x = way_pts[p2_idx][0]
                        point2.y = way_pts[p2_idx][1]
                        point2.z = 0.0

                        line.points.append(point1)
                        line.points.append(point2)
                        line_pub.publish(line)
                        time.sleep(0.1)

                        perimeter = cv2.arcLength(contour, True)
                        perimeter = ((perimeter / 2) + 10) * 0.05

                        if p1_idx not in graph:
                            graph[p1_idx] = {
                                "node_pos": way_pts[p1_idx],
                                "outward_edges": {p2_idx: perimeter},
                            }
                        else:
                            graph[p1_idx]["outward_edges"][p2_idx] = perimeter

                        if p2_idx not in graph:
                            graph[p2_idx] = {
                                "node_pos": way_pts[p2_idx],
                                "outward_edges": {p1_idx: perimeter},
                            }
                        else:
                            graph[p2_idx]["outward_edges"][p1_idx] = perimeter

                elif press_count == 2:
                    for i in range(0, 5):
                        print("")
                    print("PRESSED SECOND TIME")
                    for i in range(0, 5):
                        print("")
                    press_count = 1

                    file = open(
                        home + "/waypoint/output_yaml/connected_graph_high_level.yaml",
                        "w",
                    )
                    yaml.dump(graph, file)
                    file.close()

                    file = open(
                        home + "/waypoint/output_yaml/way_pts_high_level.yaml", "w"
                    )
                    yaml.dump(way_pts, file)
                    file.close()

                    file = open(
                        home + "/waypoint/output_yaml/edges_high_level.yaml", "w"
                    )
                    yaml.dump(lines, file)
                    file.close()
                    press_count += 1

            if load == 1:
                for i in range(0, 5):
                    print("")
                print(" SAVED ")
                for i in range(0, 5):
                    print("")
                press_count = 1

                file = open(
                    home + "/waypoint/output_yaml/connected_graph__edited.yaml", "w"
                )
                yaml.dump(graph, file)
                file.close()

                file = open(home + "/waypoint/output_yaml/way_pts_edited.yaml", "w")
                yaml.dump(way_pts, file)
                file.close()

                file = open(home + "/waypoint/output_yaml/edges_edited.yaml", "w")
                yaml.dump(lines, file)

                server.clear()
                server.applyChanges()

                line = Marker()
                line.header.frame_id = "map"
                line.ns = "edges"
                line.action = line.DELETEALL
                line_pub.publish(line)
                file.close()

        if feedback.menu_entry_id == 6:
            global lines
            global way_pts
            global graph
            global load
            load = 1
            way_pts = {}
            print("")
            print("")
            print("")
            print("LOAD PREVIOUS WORK")
            print("")
            print("")
            print("")
            server.clear()
            server.applyChanges()

            line = Marker()
            line.header.frame_id = "map"
            line.ns = "edges"
            line.action = line.DELETEALL
            line_pub.publish(line)

            lines_path = raw_input("Enter path for edges.yaml :")
            graph_path = raw_input("Enter path for connected_graph.yaml :")
            way_pts_path = raw_input("Enter path for way_pts.yaml :")

            if lines_path == "":
                lines_path = home + "/waypoint/output_yaml/edges_high_level.yaml"
            if graph_path == "":
                graph_path = (
                    home + "/waypoint/output_yaml/connected_graph_high_level.yaml"
                )
            if way_pts_path == "":
                way_pts_path = home + "/waypoint/output_yaml/way_pts_high_level.yaml"

            with open(lines_path, "r") as stream:
                lines = yaml.load(stream)

            with open(graph_path, "r") as stream:
                graph = yaml.load(stream)

            with open(way_pts_path, "r") as stream:
                way_pts = yaml.load(stream)

            for i in way_pts:
                x = way_pts[i][0]
                y = way_pts[i][1]
                position = Point(x, y, 0)
                make6DofMarker(
                    False, InteractiveMarkerControl.NONE, position, True, name=str(i)
                )
                server.applyChanges()
            print("way_pts=", way_pts)

            for i in lines:
                line = Marker()
                line.header.frame_id = "map"
                line.ns = "edges"
                line.type = Marker.LINE_STRIP
                line.header.stamp = rospy.Time.now()
                line.action = line.ADD
                line.scale.x = 0.3
                line.color.r = 0.0
                line.color.g = 1.0
                line.color.b = 0.0
                line.color.a = 1.0
                line.id = i[2]

                print(i, lines[i])
                print("")
                print("")
                p1_x = lines[i][0][0]
                p1_y = lines[i][0][1]
                p2_x = lines[i][1][0]
                p2_y = lines[i][1][1]
                point1 = Point(p1_x, p1_y, 0)

                point2 = Point(p2_x, p2_y, 0)

                line.points.append(point1)
                line.points.append(point2)
                line_pub.publish(line)
                time.sleep(0.2)
            print("LENGTH :", len(lines))

        rospy.loginfo(
            s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "."
        )
    elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
        rospy.loginfo(s + ": pose changed")

    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
        rospy.loginfo(s + ": mouse down" + mp + ".")
    elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
        rospy.loginfo(s + ": mouse up" + mp + ".")
    server.applyChanges()


def alignMarker(feedback):
    pose = feedback.pose

    pose.position.x = round(pose.position.x - 0.5) + 0.5
    pose.position.y = round(pose.position.y - 0.5) + 0.5

    rospy.loginfo(
        feedback.marker_name
        + ": aligning position = "
        + str(feedback.pose.position.x)
        + ","
        + str(feedback.pose.position.y)
        + ","
        + str(feedback.pose.position.z)
        + " to "
        + str(pose.position.x)
        + ","
        + str(pose.position.y)
        + ","
        + str(pose.position.z)
    )

    server.setPose(feedback.marker_name, pose)
    server.applyChanges()


def rand(min_, max_):
    return min_ + random() * (max_ - min_)


def makeBox(msg):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 1
    marker.color.g = 0
    marker.color.b = 0
    marker.color.a = 1.0

    return marker


def makeBoxControl(msg):
    control = InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append(makeBox(msg))
    msg.controls.append(control)
    return control


def saveMarker(int_marker):
    server.insert(int_marker, processFeedback)


def make6DofMarker(fixed, interaction_mode, position, show_6dof=False, name=""):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "map"
    int_marker.pose.position = position
    int_marker.scale = 5

    int_marker.name = name
    int_marker.description = name

    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = {
            InteractiveMarkerControl.MOVE_3D: "MOVE_3D",
            InteractiveMarkerControl.ROTATE_3D: "ROTATE_3D",
            InteractiveMarkerControl.MOVE_ROTATE_3D: "MOVE_ROTATE_3D",
        }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof:
            int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]

    if show_6dof:
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)


def makeMenuMarker(position):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "base_link"
    int_marker.pose.position = position
    int_marker.scale = 1

    int_marker.name = "context_menu"
    int_marker.description = "Context Menu\n(Right Click)"

    control = InteractiveMarkerControl()
    control.interaction_mode = InteractiveMarkerControl.MENU
    control.description = "Options"
    control.name = "menu_only_control"
    int_marker.controls.append(copy.deepcopy(control))

    marker = makeBox(int_marker)
    control.markers.append(marker)
    control.always_visible = True
    int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply(server, int_marker.name)


def callback(msg):
    global graph
    rospy.loginfo("coordinates:x=%f y=%f" % (msg.point.x, msg.point.y))
    name_of_waypt = max([k for k in way_pts]) + 1
    way_pts[name_of_waypt] = [msg.point.x, msg.point.y]
    print("added waypoint :", name_of_waypt)
    position = Point(msg.point.x, msg.point.y, 0)
    make6DofMarker(
        False, InteractiveMarkerControl.NONE, position, True, name=str(name_of_waypt)
    )
    graph[name_of_waypt] = {"node_pos": way_pts[name_of_waypt], "outward_edges": {}}
    server.applyChanges()


home = os.path.expanduser("~")
path = home + "/Downloads/waypoint/input_images/skeleton_neom.png"
print(path)
bw_map = cv2.imread(home + "/Downloads/waypoint/input_images/skeleton_neom.png")

global im
load = 0

gray = cv2.cvtColor(bw_map, cv2.COLOR_BGR2GRAY)
im = cv2.threshold(bw_map, 100, 255, cv2.THRESH_BINARY_INV)[1]
im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
im = cv2.flip(im, 0)
corners = cv2.goodFeaturesToTrack(im, 1000, 0.375, 50)
corners = np.int0(corners)
global way_pts
way_pts = {}
global index
index = 0

for i in corners:
    x, y = i.ravel()
    way_pts[index] = [float(x * 0.05 - 10.0), float(y * 0.05 - 10.0)]
    index += 1


if __name__ == "__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    rospy.point_pub = rospy.Subscriber("/clicked_point", PointStamped, callback)

    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert("Delete waypoint", callback=processFeedback)
    menu_handler.insert("Make connected graph", callback=processFeedback)
    menu_handler.insert("Add edge", callback=processFeedback)
    menu_handler.insert("Delete Edge", callback=processFeedback)
    menu_handler.insert("Save to YAML", callback=processFeedback)
    menu_handler.insert("Load previous", callback=processFeedback)

    index = 0

    for i in way_pts:
        x = way_pts[i][0]
        y = way_pts[i][1]
        position = Point(x, y, 0)
        make6DofMarker(
            False, InteractiveMarkerControl.NONE, position, True, name=str(index)
        )
        index += 1

    print(len(way_pts))
    server.applyChanges()
    rospy.spin()
