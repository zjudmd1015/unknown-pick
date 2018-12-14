#!/usr/bin/env python
import rospy
import shape_msgs.msg as shape_msgs
import geometry_msgs.msg as geometry_msgs
import moveit_msgs.msg as moveit_msgs


def BasicMicoEnvironment():
    ## all objects in one message
    env_objects = moveit_msgs.CollisionObject()
    env_objects.header.stamp = rospy.Time.now()
    env_objects.header.frame_id = "/world"
    env_objects.id = "table_camera_user"

    #table
    # table = shape_msgs.SolidPrimitive()
    # table.type = shape_msgs.SolidPrimitive.BOX
    # table.dimensions.append(2.0)
    # y = 1.2
    # table.dimensions.append(y)
    # table.dimensions.append(0.05)

    # table_pose = geometry_msgs.Pose()
    # table_pose.position.y = - y/2.0
    # table_pose.position.z = - 0.03
    # table_pose.orientation.w = 1.0

    #camera
    camera = shape_msgs.SolidPrimitive()
    camera.type = shape_msgs.SolidPrimitive.BOX
    camera.dimensions.append(0.3)
    camera.dimensions.append(0.08)
    camera.dimensions.append(0.06)

    camera_pose = geometry_msgs.Pose()
    camera_pose.position.x = 0.581
    camera_pose.position.y = 0.065
    camera_pose.position.z = 0.05 / 2.0 + 0.376
    camera_pose.orientation.z =  0.3827
    camera_pose.orientation.w = -0.9239


    #camera pole
    camera_pole = shape_msgs.SolidPrimitive()
    camera_pole.type = shape_msgs.SolidPrimitive.CYLINDER
    camera_pole.dimensions.append(0.376) #height
    camera_pole.dimensions.append(0.02) #radius

    camera_pole_pose = geometry_msgs.Pose()
    camera_pole_pose.position.x = 0.581
    camera_pole_pose.position.y = 0.065
    camera_pole_pose.position.z = 0.376 / 2.0
    camera_pole_pose.orientation.w = 1.0

    #user area
    user_area = shape_msgs.SolidPrimitive()
    user_area.type = shape_msgs.SolidPrimitive.BOX
    user_area.dimensions.append(1.2)
    user_area.dimensions.append(0.05)
    user_area.dimensions.append(0.6)

    user_area_pose = geometry_msgs.Pose()
    user_area_pose.position.x = 0.0
    user_area_pose.position.y = 0.3
    user_area_pose.position.z = 0.3
    user_area_pose.orientation.w = 1.0

    # env_objects.primitives.append(table) #one can also append more objects under the same id
    env_objects.primitives.append(camera)
    env_objects.primitives.append(camera_pole)
    env_objects.primitives.append(user_area)

    # env_objects.primitive_poses.append(table_pose)
    env_objects.primitive_poses.append(camera_pose)
    env_objects.primitive_poses.append(camera_pole_pose)
    env_objects.primitive_poses.append(user_area_pose)

    env_objects.operation = moveit_msgs.CollisionObject.ADD

    pub_coll_obj = rospy.Publisher('/collision_object', moveit_msgs.CollisionObject, queue_size = 1)
    count = 0
    rate = rospy.Rate(50)
    while not rospy.is_shutdown() and count < 100:
        pub_coll_obj.publish(env_objects)
        count += 1
        rate.sleep()



def main():

    rospy.init_node("basic_mico_environment")

    BasicMicoEnvironment()



if __name__=='__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
