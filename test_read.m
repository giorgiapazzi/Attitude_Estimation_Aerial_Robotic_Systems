%% read and plot rosbag data

%set filename
filename = '2022-12-22-15-22-04_primo_test.bag';

%read file:
rbr = rosbagreader(filename);

%do some preliminary checks on contents
if 0,
    rbr.AvailableTopics
    %returns:
    %     /diagnostics                                                3928       diagnostic_msgs/DiagnosticArray          {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵diagnostic_msgs/DiagnosticStatus[] Status↵  int8 OK↵  int8 WARN↵  int8 ERROR↵  int8 STALE↵  int8 Level↵  char Name↵  char Message↵  char HardwareId↵  diagnostic_msgs/KeyValue[] Values↵    char Key↵    char Value↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              }
    %     /image_view/output                                          1871       sensor_msgs/Image                        {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵uint32 Height↵uint32 Width↵char Encoding↵uint8 IsBigendian↵uint32 Step↵uint8[] Data↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 }
    %     /image_view/parameter_descriptions                             1       dynamic_reconfigure/ConfigDescription    {'dynamic_reconfigure/Group[] Groups↵  char Name↵  char Type↵  dynamic_reconfigure/ParamDescription[] Parameters↵    char Name↵    char Type↵    uint32 Level↵    char Description↵    char EditMethod↵  int32 Parent↵  int32 Id↵dynamic_reconfigure/Config Max↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵dynamic_reconfigure/Config Min↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵dynamic_reconfigure/Config Dflt↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵'}
    %     /image_view/parameter_updates                                  1       dynamic_reconfigure/Config               {'dynamic_reconfigure/BoolParameter[] Bools↵  char Name↵  logical Value↵dynamic_reconfigure/IntParameter[] Ints↵  char Name↵  int32 Value↵dynamic_reconfigure/StrParameter[] Strs↵  char Name↵  char Value↵dynamic_reconfigure/DoubleParameter[] Doubles↵  char Name↵  double Value↵dynamic_reconfigure/GroupState[] Groups↵  char Name↵  logical State↵  int32 Id↵  int32 Parent↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 }
    %     /imu/data                                                   1871       sensor_msgs/Imu                          {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵geometry_msgs/Quaternion Orientation↵  double X↵  double Y↵  double Z↵  double W↵double[9] OrientationCovariance↵geometry_msgs/Vector3 AngularVelocity↵  double X↵  double Y↵  double Z↵double[9] AngularVelocityCovariance↵geometry_msgs/Vector3 LinearAcceleration↵  double X↵  double Y↵  double Z↵double[9] LinearAccelerationCovariance↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  }
    %     /imu/mag                                                    1871       sensor_msgs/MagneticField                {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵geometry_msgs/Vector3 MagneticField↵  double X↵  double Y↵  double Z↵double[9] MagneticFieldCovariance↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               }
    %     /imu_data_str                                               3742       std_msgs/String                          {'char Data↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 }
    %     /pressure                                                   1871       sensor_msgs/FluidPressure                {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵double FluidPressure↵double Variance↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   }
    %     /rosout                                                       11       rosgraph_msgs/Log                        {'int8 DEBUG↵int8 INFO↵int8 WARN↵int8 ERROR↵int8 FATAL↵std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵int8 Level↵char Name↵char Msg↵char File↵char Function↵uint32 Line↵char[] Topics↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             }
    %     /tf                                                        37031       tf2_msgs/TFMessage                       {'geometry_msgs/TransformStamped[] Transforms↵  std_msgs/Header Header↵    uint32 Seq↵    Time Stamp↵    char FrameId↵  char ChildFrameId↵  geometry_msgs/Transform Transform↵    geometry_msgs/Vector3 Translation↵      double X↵      double Y↵      double Z↵    geometry_msgs/Quaternion Rotation↵      double X↵      double Y↵      double Z↵      double W↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 }
    %     /time_reference                                             3741       sensor_msgs/TimeReference                {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵Time TimeRef↵char Source↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               }
    %     /usb_cam/camera_info                                        1871       sensor_msgs/CameraInfo                   {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵uint32 Height↵uint32 Width↵char DistortionModel↵double[] D↵double[9] K↵double[9] R↵double[12] P↵uint32 BinningX↵uint32 BinningY↵sensor_msgs/RegionOfInterest Roi↵  uint32 XOffset↵  uint32 YOffset↵  uint32 Height↵  uint32 Width↵  logical DoRectify↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          }
    %     /usb_cam/image_raw                                          1871       sensor_msgs/Image                        {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵uint32 Height↵uint32 Width↵char Encoding↵uint8 IsBigendian↵uint32 Step↵uint8[] Data↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 }
    %     /usb_cam/image_raw/compressed                               1871       sensor_msgs/CompressedImage              {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵char Format↵uint8[] Data↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                               }
    %     /usb_cam/image_raw/compressed/parameter_descriptions           1       dynamic_reconfigure/ConfigDescription    {'dynamic_reconfigure/Group[] Groups↵  char Name↵  char Type↵  dynamic_reconfigure/ParamDescription[] Parameters↵    char Name↵    char Type↵    uint32 Level↵    char Description↵    char EditMethod↵  int32 Parent↵  int32 Id↵dynamic_reconfigure/Config Max↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵dynamic_reconfigure/Config Min↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵dynamic_reconfigure/Config Dflt↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵'}
    %     /usb_cam/image_raw/compressed/parameter_updates                1       dynamic_reconfigure/Config               {'dynamic_reconfigure/BoolParameter[] Bools↵  char Name↵  logical Value↵dynamic_reconfigure/IntParameter[] Ints↵  char Name↵  int32 Value↵dynamic_reconfigure/StrParameter[] Strs↵  char Name↵  char Value↵dynamic_reconfigure/DoubleParameter[] Doubles↵  char Name↵  double Value↵dynamic_reconfigure/GroupState[] Groups↵  char Name↵  logical State↵  int32 Id↵  int32 Parent↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 }
    %     /usb_cam/image_raw/theora                                   1874       theora_image_transport/Packet            {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵uint8[] Data↵int32 BOS↵int32 EOS↵int64 Granulepos↵int64 Packetno↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     }
    %     /usb_cam/image_raw/theora/parameter_descriptions               1       dynamic_reconfigure/ConfigDescription    {'dynamic_reconfigure/Group[] Groups↵  char Name↵  char Type↵  dynamic_reconfigure/ParamDescription[] Parameters↵    char Name↵    char Type↵    uint32 Level↵    char Description↵    char EditMethod↵  int32 Parent↵  int32 Id↵dynamic_reconfigure/Config Max↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵dynamic_reconfigure/Config Min↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵dynamic_reconfigure/Config Dflt↵  dynamic_reconfigure/BoolParameter[] Bools↵    char Name↵    logical Value↵  dynamic_reconfigure/IntParameter[] Ints↵    char Name↵    int32 Value↵  dynamic_reconfigure/StrParameter[] Strs↵    char Name↵    char Value↵  dynamic_reconfigure/DoubleParameter[] Doubles↵    char Name↵    double Value↵  dynamic_reconfigure/GroupState[] Groups↵    char Name↵    logical State↵    int32 Id↵    int32 Parent↵'}
    %     /usb_cam/image_raw/theora/parameter_updates                    1       dynamic_reconfigure/Config               {'dynamic_reconfigure/BoolParameter[] Bools↵  char Name↵  logical Value↵dynamic_reconfigure/IntParameter[] Ints↵  char Name↵  int32 Value↵dynamic_reconfigure/StrParameter[] Strs↵  char Name↵  char Value↵dynamic_reconfigure/DoubleParameter[] Doubles↵  char Name↵  double Value↵dynamic_reconfigure/GroupState[] Groups↵  char Name↵  logical State↵  int32 Id↵  int32 Parent↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 }
    %     /velocity                                                   1871       geometry_msgs/TwistStamped               {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵geometry_msgs/Twist Twist↵  geometry_msgs/Vector3 Linear↵    double X↵    double Y↵    double Z↵  geometry_msgs/Vector3 Angular↵    double X↵    double Y↵    double Z↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                            }
    %     /vicon/markers                                             37030       vicon_bridge/Markers                     {'std_msgs/Header Header↵  uint32 Seq↵  Time Stamp↵  char FrameId↵uint32 FrameNumber↵vicon_bridge/Marker[] Markers↵  char MarkerName↵  char SubjectName↵  char SegmentName↵  geometry_msgs/Point Translation↵    double X↵    double Y↵    double Z↵  logical Occluded↵'                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              }
    %     /vicon/zeno_wand/zeno_wand 
    %
    % of witch of interests here are:
    % 
    %     /imu/data, /imu/mag, /vicon/zeno_wand/zeno_wand 

end

%extract relevant data 
t_start = rbr.StartTime; %NOTE: t_start will be used as unique starting time offset !!!
t_end = rbr.EndTime;

% vicon :
vicon_subset = select(rbr,...
    Time=[t_start t_end],...
    Topic='/vicon/zeno_wand/zeno_wand');
% vicon position:
ts_vicon_pos = timeseries(vicon_subset,'Transform.Translation.X','Transform.Translation.Y','Transform.Translation.Z');
ts_vicon_pos.Time = ts_vicon_pos.Time - t_start;
% vicon quaternion:
ts_vicon_quat = timeseries(vicon_subset,'Transform.Rotation.X','Transform.Rotation.Y','Transform.Rotation.Z','Transform.Rotation.W');
ts_vicon_quat.Time = ts_vicon_quat.Time - t_start;


% imu :
imu_subset = select(rbr,...
    Time=[t_start t_end],...
    Topic='/imu/data');
% imu gyro:
ts_imu_gyro = timeseries(imu_subset,'AngularVelocity.X','AngularVelocity.Y','AngularVelocity.Z');
ts_imu_gyro.Time = ts_imu_gyro.Time - t_start;
% imu acc:
ts_imu_acc = timeseries(imu_subset,'LinearAcceleration.X','LinearAcceleration.Y','LinearAcceleration.Z');
ts_imu_acc.Time = ts_imu_acc.Time - t_start;
% imu quat:
ts_imu_quat = timeseries(imu_subset,'Orientation.X','Orientation.Y','Orientation.Z','Orientation.W');
ts_imu_quat.Time = ts_imu_quat.Time - t_start;
%mag subset 
imu_subset = select(rbr,...
    Time=[t_start t_end],...
    Topic='/imu/mag');
% imu mag: 
ts_imu_mag = timeseries(imu_subset,'MagneticField_.X','MagneticField_.Y','MagneticField_.Z');
ts_imu_mag.Time = ts_imu_mag.Time - t_start;

%camera subset
%get camera info and extract a timeseries only to obtain vector of
%times!!!!
cam_subset = select(rbr,...
    Time=[t_start t_end],...
    Topic='/usb_cam/camera_info');
%create a timeseries with times only:
ts_cam_times = timeseries(cam_subset,'Roi.XOffset');
ts_cam_times.Time = ts_cam_times.Time - t_start;
ts_cam_times.Data = ts_cam_times.Time;
% now ts_cam_times can be used to find the closet camera image to current
% time 
%now create a new cam_subset for image/raw
cam_subset = select(rbr,...
    Time=[t_start t_end],...
    Topic='/usb_cam/image_raw');
%this will be used to extract single images using readMessages and
%selecting a single message at a time



%% convert quaternions to euler
% notes:
% - using the matlab function rotm = quat2rotm(quat)
%   returns a rotation matrix from body to nav 
% - quaternions in matlab must be [w x y z]
% - using rotationMatrix = rotmat(quat,'frame')
%   creates a rotation matrix that rotates nav to body
% - using quat = quaternion(RM,'rotmat','frame') creates a quaternion from rotation matrix RM for
%   frame rotations
% - using quat = quaternion(RM,'rotmat','point') creates a quaternion for
%   point rotations
% - using eul = rotm2eul(rotm,'ZYX') converts the rotation matrix rotm to euler
%   angles BUT requires rotm to be a body to nav matrix AND returns yaw, pitch, roll  in this order !!!

%test 1 : convert vicon quaternions to euler angles 
ts_vicon_euler = ts_vicon_pos; %simply init the timeserires with a 3 element vector 
for idx = 1:length(ts_vicon_quat.Data),
    %get quaternion in w,x,y,z format :
    q_in = quaternion(ts_vicon_quat.Data(idx,[4 1 2 3]));
    rm = rotmat(q_in,'frame'); %this rot mat transforms nav to body
    eul_out = rotm2eul(rm','ZYX');
    eul_out = eul_out([3 2 1]); %adjust for order of angles returned by rotm2eul
    %fill in new timeseries
    ts_vicon_euler.Data(idx,:) = eul_out;
end
if 0,
    figure;
    plot(ts_vicon_euler.Time, ts_vicon_euler.Data)

end

%NOTA: ci sono 4 sistemi di riferimento:
%   - IMU (il frame base della IMU - suppongo allineato come NWU) 
%   - imu (il frame assi corpo della IMU)
%   - VICON (il frame di riferiemnto vicon: asse X puntao verso il mobile,
%           Y verso la finestra, Z in alto)
%   - assi corpo wand (con l'oggetto zeno_wand l'asse X, lungo l'asse corto
%           della wand punta verso sinistra, l'asse Y punta verso la manopola,
%           l'asse Z verso l'alto)

%test 2: convert IMU quternions to vicon quaternions and compare
%
% define IMU mounting:
%  - the IMU X axis is aligned with -Y wand (vicon) axis
%  - the IMU Y axis is aligned with -X wand (vicon) axis 
% thus the rotation from IMU axis to wand_axis is : 
% first Rz(90), then Ry(180) 
roll = 0;
pitch = pi;
yaw = pi/2;
Rotx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
Roty = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
Rotz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
r_imu_mount = Rotx*Roty*Rotz; %this is C_imu^wand
% in order to compute rot matrix from base_vicon to wand_vicon using imu
% data for comparison the trasnsformation is al follows:
% C_b_VICON^wand = C_imu^wand * C_b_IMU^imu *C_b_IMU^b_VICON ^T
%
% NOTA: al momento non so la trasformazione da frame base IMU a frame base VICON ... immagino
% dipenda anche dalla calibrazione del magnetometro 
% => provo a stimarla come: 
% C_b_VICON^wand * C_b_IMU^b_VICON = C_imu^wand * C_b_IMU^imu 
% C_b_IMU^b_VICON = C_b_VICON^wand ^T * C_imu^wand * C_b_IMU^imu
% 
% provo a graficare gli angoli di eulero di C_b_IMU^b_VICON che dovrebbero
% essere costanti in teoria (nel caso di stima esatta dell'assetto sia di IMU che di VICON 
ts_IMU_VICON = ts_imu_acc; %simply init the timeseries with a 3 element vector 
for idx = 1:length(ts_imu_quat.Data),
    %get quaternion in w,x,y,z format :
    q_in = quaternion(ts_imu_quat.Data(idx,[4 1 2 3]));
    C_IMU_imu = rotmat(q_in,'frame'); %this rot mat transforms IMU_base_frame to imu_body_frame (C_b_IMU^imu)
    %find corresponding vicon sample
    t_imu_temp = ts_IMU_VICON.Time(idx);
    [m_discard,idx_best_time] = min(abs(ts_vicon_quat.Time-t_imu_temp));
    %get C_b_VICON^wand
    q_in = quaternion(ts_vicon_quat.Data(idx_best_time,[4 1 2 3]));
    C_VICON_wand = rotmat(q_in,'frame'); %this rot mat transforms nav to body
    %compute C_b_IMU^b_VICON
    % C_b_IMU^b_VICON = C_b_VICON^wand ^T * C_imu^wand * C_b_IMU^imu
    C_IMU_VICON = C_VICON_wand' * r_imu_mount * C_IMU_imu;
    %convert to euler angles
    eul_out = rotm2eul(C_IMU_VICON','ZYX');
    eul_out = eul_out([3 2 1]); %adjust for order of angles returned by rotm2eul
    %fill in new timeseries
    ts_IMU_VICON.Data(idx,:) = eul_out;
end
if 0,
    figure;
    plot(ts_IMU_VICON.Time, ts_IMU_VICON.Data*180/pi)
    % dai dati pare che roll e pitch della trasformazione ts_IMU_VICON
    % siano più o meno costanti :oscillando di +- 0.5 deg con picchi a 1.5
    % gradi (escludendo un picco unico di 10 gradi)
    % lo yaw invece varia lentamente nel tempo tra -82 e -92 gradi 
    % per cui prendo un valore intermedio a -86 gradi 
    %
    %TODO: verificare la IMU xsens cosa usa come riferimento per lo yaw 
    % visto che -86 gradi non paiono corretti... 
    % l'asse X di vicon punta verso il muro di destra (entrando) ed il NORD
    % è girato verso il muro di entrata e forma un angolo di circa 66 gradi 
    % per cui la rotaz da IMU a VICON, considerando la Z up si ottiene
    % ruotando di +66 gradi (non -86)
end
%
%
% costruisco la C_IMU_VICON a partire dai dati medi sperimentali:
roll = 0;
pitch = 0;
yaw = -86*pi/180;
Rotx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
Roty = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
Rotz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
C_IMU_VICON = Rotx*Roty*Rotz;
%
%ora posso convertire gli angoli stimati dalla IMU rispetto al frame base IMU 
% in angoli rispetto al frame VICON e confronarli direttamente
%
% in order to compute rot matrix from base_vicon to wand_vicon using imu
% data for comparison the trasnsformation is al follows:
% C_b_VICON^wand = C_imu^wand * C_b_IMU^imu *C_b_IMU^b_VICON ^T
%
ts_VICON_wand = ts_imu_acc; %simply init the timeseries with a 3 element vector 
for idx = 1:length(ts_imu_quat.Data),
    %get quaternion in w,x,y,z format :
    q_in = quaternion(ts_imu_quat.Data(idx,[4 1 2 3]));
    C_IMU_imu = rotmat(q_in,'frame'); %this rot mat transforms IMU_base_frame to imu_body_frame (C_b_IMU^imu)
    % calcolo C_b_VICON^wand = C_imu^wand * C_b_IMU^imu *C_b_IMU^b_VICON ^T
    C_VICON_wand = r_imu_mount * C_IMU_imu * C_IMU_VICON';
    %convert to euler angles
    eul_out = rotm2eul(C_VICON_wand','ZYX');
    eul_out = eul_out([3 2 1]); %adjust for order of angles returned by rotm2eul
    %fill in new timeseries
    ts_VICON_wand.Data(idx,:) = eul_out;
end
if 0,
    figure;
    plot(ts_VICON_wand.Time, ts_VICON_wand.Data*180/pi)
end
if 1,
    %compare imu attitude with vicon data
    figure;
    plot(ts_VICON_wand.Time, ts_VICON_wand.Data*180/pi);
    hold on
    grid on
    plot(ts_vicon_euler.Time, ts_vicon_euler.Data*180/pi)
    legend('roll (vicon)','pitch (vicon)','yaw (vicon)', 'roll (imu)','pitch (imu)','yaw (imu)');
    hold off
    %NOTA:  qui ts_vicon_euler contiene roll pitch e yaw ottenuti da
    %       quaternioni di vicon
    %       ts_VICON_wand contiene invece roll pitch e yaw ottenuti
    %       dall'assetto stimato dalla IMU ruotato nel frame vicon 
end

%NOTA:  se si vuole ruotare il vettore v_imu, acquisito in assi corpo imu (l'accelerazione ad esempio) 
%       e trasformala nel frame della wand (compensando anche per il montaggio della IMU 
%       come se la imu quindi fosse stata allineata con gli assi wand) si deve fare:
%           v_wand = C_imu_wand *v_imu 
%       con 
%           C_imu_wand = r_imu_mount !!!!
%       
%       se poi si vuole tale vettore in assi "navigation" di VICON
%       utilizzando roll pitch e yaw della IMU
%           V_VICON = C_imu_VICON * v_imu
%       con 
%           C_imu_VICON = C_IMU_VICON * C_IMU_imu(r,p,y)^T
%       se invece si vuole usare l'assetto di VICON per farlo
%       si puo' prendere:
%           C_imu_VICON = C_VICON^wand(r,p,y)^T * C_imu_wand
%

%% test plots 

%test vicon position data
if 0,
    figure;
    subplot(3,1,1);
    plot(ts_vicon_pos.Time, ts_vicon_pos.Data(:,1)); grid on;
    subplot(3,1,2);
    plot(ts_vicon_pos.Time, ts_vicon_pos.Data(:,2)); grid on;
    subplot(3,1,3);
    plot(ts_vicon_pos.Time, ts_vicon_pos.Data(:,3)); grid on;
    
    figure;
    plot3(ts_vicon_pos.Data(:,1), ts_vicon_pos.Data(:,2), ts_vicon_pos.Data(:,3));
    axis equal;
    grid on;
    xlabel('X'), ylabel('Y'), zlabel('Z');


end






%test imu data (in imu body frame)
if 0,
    figure;
    subplot(3,1,1);
    plot(ts_imu_acc.Time, ts_imu_acc.Data(:,1:3));
    subplot(3,1,2);
    plot(ts_imu_gyro.Time, ts_imu_gyro.Data(:,1:3));
    subplot(3,1,3);
    plot(ts_imu_mag.Time, ts_imu_mag.Data(:,1:3));

    %compare quaternions (without any conversion)
    figure;
    subplot(2,1,1);
    plot(ts_imu_quat.Time, ts_imu_quat.Data(:,1:4));
    subplot(2,1,2);
    plot(ts_vicon_quat.Time, ts_vicon_quat.Data(:,1:4));
   

end


%% animate trajectory 
% note: set a callback when a key is pressed using: https://it.mathworks.com/matlabcentral/answers/561245-how-to-detect-a-keyboard-key-in-a-while-loop-using-a-figure
if 0,
    hf = figure;
    set(hf,'KeyPressFcn', @myKeyPressFcn,'UserData', false);
    %set axis length
    AL = 0.25;
    %set skip frames
    SF = 10;
    %create handles for trajectory and axes
    ht = plot3(ts_vicon_pos.Data(1,1),ts_vicon_pos.Data(1,2),ts_vicon_pos.Data(1,3),'b-');
    hold on;
    grid on;
    ho = plot3(ts_vicon_pos.Data(1,1),ts_vicon_pos.Data(1,2),ts_vicon_pos.Data(1,3),'or');
    hax = plot3(0,0,0,'r-','LineWidth',3);
    hay = plot3(0,0,0,'g-','LineWidth',3);    
    haz = plot3(0,0,0,'b-','LineWidth',3);
    %set dimensions 
    axis equal;
    axis([min(ts_vicon_pos.Data(:,1)), max(ts_vicon_pos.Data(:,1)),...
        min(ts_vicon_pos.Data(:,2)), max(ts_vicon_pos.Data(:,2)),...
        min(ts_vicon_pos.Data(:,3)), max(ts_vicon_pos.Data(:,3))]);
    grid on;
    xlabel('X'), ylabel('Y'), zlabel('Z');


    %iterate 
    idx = 2;
    while idx <= length(ts_vicon_pos.Data),
    %for idx = 2:SF:length(ts_vicon_pos.Data),
        %set time in title 
        title('Time: ',num2str(ts_vicon_pos.Time(idx,1)));
        % adjust trajectory 
        set(ht,'XData',ts_vicon_pos.Data(1:idx,1), 'YData',ts_vicon_pos.Data(1:idx,2),'ZData',ts_vicon_pos.Data(1:idx,3));
        %adjust frame origin 
        set(ho,'XData',ts_vicon_pos.Data(idx,1), 'YData',ts_vicon_pos.Data(idx,2),'ZData',ts_vicon_pos.Data(idx,3));
        %adjust axes
        % compute rotmatrix
        roll = ts_vicon_euler.Data(idx,1);
        pitch = ts_vicon_euler.Data(idx,2);
        yaw = ts_vicon_euler.Data(idx,3);
        Rotx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
        Roty = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
        Rotz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
        rm = Rotx*Roty*Rotz; %rm totates nav to body, 
        % must use its trasnpose to obtain body coordinate axis into nav frame for plottting
        axv = rm'*[AL 0 0]';
        ayv = rm'*[0 AL 0]';
        azv = rm'*[0 0 AL]';
        set(hax,'XData',[ts_vicon_pos.Data(idx,1) ts_vicon_pos.Data(idx,1)+axv(1)], ...
            'YData',[ts_vicon_pos.Data(idx,2) ts_vicon_pos.Data(idx,2)+axv(2)],...
            'ZData',[ts_vicon_pos.Data(idx,3) ts_vicon_pos.Data(idx,3)+axv(3)]);
        set(hay,'XData',[ts_vicon_pos.Data(idx,1) ts_vicon_pos.Data(idx,1)+ayv(1)], ...
            'YData',[ts_vicon_pos.Data(idx,2) ts_vicon_pos.Data(idx,2)+ayv(2)],...
            'ZData',[ts_vicon_pos.Data(idx,3) ts_vicon_pos.Data(idx,3)+ayv(3)]);
        set(haz,'XData',[ts_vicon_pos.Data(idx,1) ts_vicon_pos.Data(idx,1)+azv(1)], ...
            'YData',[ts_vicon_pos.Data(idx,2) ts_vicon_pos.Data(idx,2)+azv(2)],...
            'ZData',[ts_vicon_pos.Data(idx,3) ts_vicon_pos.Data(idx,3)+azv(3)]);

        %manage plotting pause
        if hf.UserData
            % in pausa
            ;
        else
            % non in pausa
            idx = idx+SF;
        end

        drawnow;
    end

end


%% animate trajectory with VIDEO !!
% note: set a callback when a key is pressed using: https://it.mathworks.com/matlabcentral/answers/561245-how-to-detect-a-keyboard-key-in-a-while-loop-using-a-figure
if 1,
    %define if an AVI video must be craeted 
    save_video = 0;
    if save_video,
        vh = VideoWriter('video_out.avi');
        open(vh);
    end

    hf = figure;
    set(hf,'KeyPressFcn', @myKeyPressFcn,'UserData', false);
    
    %prepare portion with trajectory
    subplot(1,2,1);
    %set axis length
    AL = 0.25;
    %set skip frames
    SF = 10;
    %create handles for trajectory and axes
    ht = plot3(ts_vicon_pos.Data(1,1),ts_vicon_pos.Data(1,2),ts_vicon_pos.Data(1,3),'b-');
    hold on;
    grid on;
    ho = plot3(ts_vicon_pos.Data(1,1),ts_vicon_pos.Data(1,2),ts_vicon_pos.Data(1,3),'or');
    hax = plot3(0,0,0,'r-','LineWidth',3);
    hay = plot3(0,0,0,'g-','LineWidth',3);    
    haz = plot3(0,0,0,'b-','LineWidth',3);
    %set dimensions 
    axis equal;
    axis([min(ts_vicon_pos.Data(:,1))-AL, max(ts_vicon_pos.Data(:,1))+AL,...
        min(ts_vicon_pos.Data(:,2))-AL, max(ts_vicon_pos.Data(:,2))+AL,...
        min(ts_vicon_pos.Data(:,3))-AL, max(ts_vicon_pos.Data(:,3))+AL]);
    grid on;
    xlabel('X'), ylabel('Y'), zlabel('Z');

    %prepare portion with video
    subplot(1,2,2);
    %get first image of the video 
    ros_img_msg = readMessages(cam_subset,1,'DataFormat','struct'); %set dataformat as struct otherwise rosReadImage does not work
    ros_img = rosReadImage(ros_img_msg{1});
    image(ros_img);
    axis equal
    axis([0 ros_img_msg{1}.Width 0 ros_img_msg{1}.Height]);

    if save_video,
        %adjust window size for video creation
        set(gcf, 'Position',  [50, 50, 50+1000, 50+500])
    end

    %iterate 
    idx = 2;
    while idx <= length(ts_vicon_pos.Data),
    %for idx = 2:SF:length(ts_vicon_pos.Data),
        %resume trajectory subplot
        subplot(1,2,1);
        %set time in title 
        title('Time: ',num2str(ts_vicon_pos.Time(idx,1)));
        % adjust trajectory 
        set(ht,'XData',ts_vicon_pos.Data(1:idx,1), 'YData',ts_vicon_pos.Data(1:idx,2),'ZData',ts_vicon_pos.Data(1:idx,3));
        %adjust frame origin 
        set(ho,'XData',ts_vicon_pos.Data(idx,1), 'YData',ts_vicon_pos.Data(idx,2),'ZData',ts_vicon_pos.Data(idx,3));
        %adjust axes
        % compute rotmatrix
        roll = ts_vicon_euler.Data(idx,1);
        pitch = ts_vicon_euler.Data(idx,2);
        yaw = ts_vicon_euler.Data(idx,3);
        Rotx = [1 0 0; 0 cos(roll) sin(roll); 0 -sin(roll) cos(roll)];
        Roty = [cos(pitch) 0 -sin(pitch); 0 1 0; sin(pitch) 0 cos(pitch)];
        Rotz = [cos(yaw) sin(yaw) 0; -sin(yaw) cos(yaw) 0; 0 0 1];
        rm = Rotx*Roty*Rotz; %rm totates nav to body, 
        % must use its trasnpose to obtain body coordinate axis into nav frame for plottting
        axv = rm'*[AL 0 0]';
        ayv = rm'*[0 AL 0]';
        azv = rm'*[0 0 AL]';
        set(hax,'XData',[ts_vicon_pos.Data(idx,1) ts_vicon_pos.Data(idx,1)+axv(1)], ...
            'YData',[ts_vicon_pos.Data(idx,2) ts_vicon_pos.Data(idx,2)+axv(2)],...
            'ZData',[ts_vicon_pos.Data(idx,3) ts_vicon_pos.Data(idx,3)+axv(3)]);
        set(hay,'XData',[ts_vicon_pos.Data(idx,1) ts_vicon_pos.Data(idx,1)+ayv(1)], ...
            'YData',[ts_vicon_pos.Data(idx,2) ts_vicon_pos.Data(idx,2)+ayv(2)],...
            'ZData',[ts_vicon_pos.Data(idx,3) ts_vicon_pos.Data(idx,3)+ayv(3)]);
        set(haz,'XData',[ts_vicon_pos.Data(idx,1) ts_vicon_pos.Data(idx,1)+azv(1)], ...
            'YData',[ts_vicon_pos.Data(idx,2) ts_vicon_pos.Data(idx,2)+azv(2)],...
            'ZData',[ts_vicon_pos.Data(idx,3) ts_vicon_pos.Data(idx,3)+azv(3)]);


        %resume image subplot
        subplot(1,2,2);
        %find closest image to current time
        data_time = ts_vicon_pos.Time(idx,1);
        [m_discard,idx_best_image] = min(abs(ts_cam_times.Time-data_time));
        idx_best_image;
        ros_img_msg = readMessages(cam_subset,idx_best_image,'DataFormat','struct'); %set dataformat as struct otherwise rosReadImage does not work
        ros_img = rosReadImage(ros_img_msg{1});
        image(ros_img);
        axis equal
        axis([0 ros_img_msg{1}.Width 0 ros_img_msg{1}.Height]);



        %manage plotting pause
        if hf.UserData
            % in pausa
            ;
        else
            % non in pausa
            idx = idx+SF;
        end

        drawnow;
        %if enabled adda  frame to the video file
        if save_video,
            writeVideo(vh,getframe(gcf));
        end

    end
    
    if save_video,
        close(vh);
    end


end

