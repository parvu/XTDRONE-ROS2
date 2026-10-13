# 这里也要记得写上planeid，后面得向地面站发送哈！！！！
uint16 plane_id

  # ####################################编队控制器期望状态###############################

  # 编队期望ned速度
float64 sp_ned_vel_x
float64 sp_ned_vel_y
float64 sp_ned_vel_z

  # 编队期望经纬度
float64 sp_latitude
float64 sp_longitude
float64 sp_altitude
float64 sp_relative_alt

  # 编队期望空速地速
float64 sp_air_speed
float64 sp_ground_speed

  # ####################################编队控制器误差状态###############################

  # NED下的位置误差
float64 err_p_n
float64 err_p_e
float64 err_p_d
float64 err_p_ne

  # 航迹轴系位置误差<与自己期望>
float64 err_pxk
float64 err_pyk
float64 err_pzk

  # 航迹系速度误差<与领机>
float64 led_fol_vxk
float64 led_fol_vyk
float64 led_fol_vzk

  # 航迹轴系速度误差<与自己期望>
float64 err_vxk
float64 err_vyk
float64 err_vzk

  # 角度误差
float64 led_fol_eta
float64 eta_deg

  # 体轴系位置误差<与自己期望>
float64 err_pxb
float64 err_pyb
float64 err_pzb

  # 体轴系速度误差<与自己期望>
float64 err_vxb
float64 err_vyb
float64 err_vzb

  # 体轴系速度误差<与领机>
float64 led_fol_vxb
float64 led_fol_vyb
float64 led_fol_vzb
