# Scout 二次开发


## 融合定位测试

### 传感器
- GNSS-RTK/GPS （libhector_gazebo_ros_gps.so)


      <gazebo>
        <plugin name="gazebo_ros_gps" filename="libhector_gazebo_ros_gps.so">
  	    <updateRate>10.0</updateRate>
  	    <frameId>base_link</frameId>
  	    <topicName>/gps/fix</topicName>
  	    <velocityTopicName>/gps/fix_velocity</velocityTopicName>
              <statusTopicName>/gps/status</statusTopicName>
  	    <offset>0 0 0</offset>
  	    <drift>0 0 0</drift>
  	    <gaussianNoise>0.05 0.05 0.05</gaussianNoise>
  	    <velocityDrift>0.0001 0.0001 0.0001</velocityDrift>
  	    <velocityGaussianNoise>0.5 0.5 0.5</velocityGaussianNoise>
  	</plugin>
    </gazebo>

- IMU (libgazebo_ros_imu_sensor.so)

      <gravity>true</gravity>
      <sensor name="imu_sensor" type="imu">
            <always_on>true</always_on>
            <update_rate>100</update_rate>
            <visualize>true</visualize>
            <topic>imu_topic_name</topic>
            <plugin filename="libgazebo_ros_imu_sensor.so" name="imu_plugin">
                <topicName>imu</topicName>
                <bodyName>imu_link</bodyName>
                <updateRateHZ>10.0</updateRateHZ>
                <gaussianNoise>0.0</gaussianNoise>
                <xyzOffset>0 0 0</xyzOffset>
                <rpyOffset>0 0 0</rpyOffset>
                <frameName>imu_link</frameName>
                <initialOrientationAsReference>false</initialOrientationAsReference>
            </plugin>
      <pose>0 0 0 0 0 0</pose>
      </sensor>


TODOs: 

- odom / world->base_link tf
- ground truths poses formatting
- finetune imu, gps, gt frequencies
