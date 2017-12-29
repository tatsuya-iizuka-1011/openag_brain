loginfo#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float64

if __name__ == '__main__':
    rospy.init_node('setup_solution')

    # Make sure that we're under an environment namespace.
    namespace = rospy.get_namespace()
    if namespace == '/':
        raise RuntimeError(
            "Cannot be run in the global namespace. Please "
            "designate an environment for this module."
        )
    global water_filled_flg, nutrient_dosered_flg, water_filled_conut, \
        nutrient_pump_count, ph_count
    water_filled_flg = 0
    nutrient_dosered_flg = 0
    water_filled_conut = 0
    nutrient_pump_count = 0
    ph_count = 0

    water_level = rospy.get_param("~water_level", 'high')
    target_ec = rospy.get_param("~target_ec", 1.0)
    ec_pump_delay = rospy.get_param("~ec_pump_delay", 0.5)
    nutirent_a_pump_value = rospy.get_param("nutirent_a_pump_value", 100)
    nutirent_b_pump_value = rospy.get_param("nutirent_b_pump_value", 100)
    target_ph = rospy.get_param("~target_ph", 6.0)
    ph_deadbandwidth = rospy.get_param("~ph_deadbandwidth", 0.2)
    last_nutrient_pump = time.time()

    nutrient_a_pub_name = 'nutrient_flora_duo_a/commanded'
    nutrient_b_pub_name = 'nutrient_flora_duo_b/commanded'
    nutirent_a_pub = rospy.Publisher(nutrient_a_pub_name, Float64, queue_size=10)
    nutirent_b_pub = rospy.Publisher(nutrient_b_pub_name, Float64, queue_size=10)

    ph_desired_name = 'water_potential_hydrogen/desired'
    ph_desired_pub = rospy.Publisher(ph_desired_name, Float64, queue_size=10)


    def water_filled_callback(item):
        global water_filled_flg, water_filled_conut
        if item.data == 0:
            water_filled_conut += 1
            if water_filled_conut > 5:
                water_filled_flg = 1
                rospy.loginfo('water filled')
                water_level_sub.unregister()

        else:
            water_filled_conut = 0
        return

    def nutrient_pump_callback(item):
        global water_filled_flg, nutrient_pump_count, last_nutrient_pump, \
            ec_pump_delay, nutrient_dosered_flg
        if water_filled_flg:
            curr_time = time.time()
            if curr_time - last_nutrient_pump > ec_pump_delay:
                last_nutrient_pump = curr_time
                if item.data < target_ec:
                    nutirent_a_pub.publish(nutirent_a_pump_value)
                    nutirent_b_pub.publish(nutirent_b_pump_value)
                    nutrient_pump_count = 0
                else:
                    nutirent_a_pub.publish(0.)
                    nutirent_b_pub.publish(0.)
                    nutrient_pump_count += 1
                    if nutrient_pump_count > 5:
                        rospy.loginfo('ec is corrected')
                        nutrient_dosered_flg = 1
                        ec_sub.unregister()
        return

    def ph_pump_callback(item):
        global water_filled_flg, nutrient_dosered_flg, ph_count, target_ph, \
            ph_deadbandwidth
        if water_filled_flg and nutrient_dosered_flg:
            if abs(target_ph - item.data) > ph_deadbandwidth:
                ph_desired_pub.publish(target_ph)
                ph_count = 0
            else:
                ph_count += 1
                if ph_count > 5:
                    rospy.loginfo('setup finish')
                    ph_sub.unregister()
        return

    water_level_state = 'water_level_{}/raw'.format(water_level)
    ec_state = 'water_electrical_conductivity/raw'
    ph_state = 'water_potential_hydrogen/raw'

    water_level_sub = rospy.Subscriber(water_level_state, Float64, water_filled_callback)
    ec_sub = rospy.Subscriber(ec_state, Float64, nutrient_pump_callback)
    ph_sub = rospy.Subscriber(ph_state, Float64, ph_pump_callback)

    rospy.spin()
