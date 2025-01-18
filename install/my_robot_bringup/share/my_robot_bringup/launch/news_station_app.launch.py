from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	ld = LaunchDescription()
	station_giskard = Node(
		package="my_cpp_pkg",
		executable="robot_news_station",
		name="robot_news_station_giskard",
		parameters=[
			{"robot_name": "giskard"}
		]
	)
	station_bb8 = Node(
		package="my_cpp_pkg",
		executable="robot_news_station",
		name="robot_news_station_bb8",
		parameters=[
			{"robot_name": "bb8"}
		]
	)
	station_daneel = Node(
		package="my_cpp_pkg",
		executable="robot_news_station",
		name="robot_news_station_daneel",
		parameters=[
			{"robot_name": "daneel"}
		]
	)
	station_jander = Node(
		package="my_cpp_pkg",
		executable="robot_news_station",
		name="robot_news_station_jander",
		parameters=[
			{"robot_name": "jander"}
		]
	)
	station_c3po = Node(
		package="my_cpp_pkg",
		executable="robot_news_station",
		name="robot_news_station_c3po",
		parameters=[
			{"robot_name": "c3po"}
		]
	)
	smartphone_node = Node(
		package="my_cpp_pkg",
		executable="smartphone"
	)

	ld.add_action(station_giskard)
	ld.add_action(station_bb8)
	ld.add_action(station_daneel)
	ld.add_action(station_jander)
	ld.add_action(station_c3po)
	ld.add_action(smartphone_node)
	return ld