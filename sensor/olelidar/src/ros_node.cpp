#include "ros_node.h"

olelidar::Ros_node::Ros_node(const ros::NodeHandle &pnh)
:pnh_(pnh),last_time_(ros::Time::now())
{
    pnh_.param("scan_ip", device_ip_str_, std::string("192.168.1.100"));
    pnh_.param("port", device_port_, 2368);
    pnh_.param("local_ip", local_ip_str_, std::string("192.168.1.10"));
    pnh_.param("transport",transport_str_,std::string("udp"));
    pnh_.param("device",product_,std::string("VF"));
    pnh_.param("frame_id",frame_id_,std::string("olelidar"));
    pnh_.param("ntp",use_ntp,false);
    scan_pub_ = pnh_.advertise<LaserScan>("scan", 10);
    
    ROS_INFO("Connecting LiDAR at : %s:%d", device_ip_str_.c_str(), device_port_);
    if (!OpenPort()) 
    {
        
        ROS_ERROR("Unable to establish connection");
        return;
    }   
    else
    {  
       
        
        seq=0;
    }
    
    is_loop=true;
    current_angle=0;
    last_angle=0;
    transport_->read(buf,packet_size);
    SelectType(buf.c_array());
    reader=std::thread(&olelidar::Ros_node::ReadPacket,this);
}

bool olelidar::Ros_node::OpenPort()
{
    //std::cout<<"start1"<<std::endl;
    info = std::make_shared<HandleInfo>();
    info->handle_type = transport_str_ == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;
    info->hostname=device_ip_str_;
    info->port=std::to_string(device_port_);
    if (info->handle_type == HandleInfo::HANDLE_TYPE_UDP)
    {
        transport_ = std::make_unique<UDPTransport>(info->hostname, info->port);       
        if(!transport_->connect())
        {
            ROS_ERROR("Unable to establish UDP connection");
            return false;
        }
        info->endpoint = transport_->get_host_ip();
        info->port = transport_->get_port();
    }
    else if (info->handle_type == HandleInfo::HANDLE_TYPE_TCP)
    {
      transport_ = std::make_unique<TCPTransport>(info->hostname);
       // if initially port was not set, request_handle sets it
      // set the urpdated port in transport
      transport_->set_port(info->port);
      if (!transport_->connect())
      {
         ROS_ERROR("Unable to establish TCP connection");
         return false;
      }
   }
    else
   {
    ROS_ERROR("Incorrect transport option");
    return false;
   }
   return true;
}

void olelidar::Ros_node::ReadPacket()
{
    while(is_loop)
    {       
      //std::cout<<"start22"<<std::endl;  
    if(!transport_->read(buf,packet_size))
    {
        ROS_ERROR("Can't receive data!");
        return;
    }           
    else
    {
        GetDataType(buf.c_array());
    } 
    }          
}

void olelidar::Ros_node::GetDataType(uint8_t* data)
{
    //std::cout<<"start3"<<std::endl;
    if(product_=="VF")
    {
        parser->read_header(data);         
        parser->get_info();
        parser->read_data((data+48),parser->scan_info.packet_size);           
        current_angle=parser->scan_info.start_index;       
        if(current_angle>=last_angle)
        {
          last_angle=current_angle;
        }
        else
        {
            last_angle=current_angle;
            Publish();
        }
    }
    else
    {
        //other lidar to do
    }

}

void olelidar::Ros_node::FillData()
{
    
}

void olelidar::Ros_node::Publish()
{
    if(parser->distance.empty())
    {
        ROS_ERROR("No data to publish!");
        return;
    }
    scanData.header.seq=seq;
    seq++;
    scanData.header.frame_id = frame_id_;
    if(use_ntp)
    {
        scanData.header.stamp=parser->scan_info.ntp;
    }
    else scanData.header.stamp=ros::Time::now();
    scanData.angle_min=deg2rad(parser->scan_info.angle_min);
    scanData.angle_max=deg2rad(parser->scan_info.angle_max);
    scanData.angle_increment=deg2rad(parser->scan_info.angle_increment);
    scanData.range_max=parser->scan_info.range_max;
    scanData.range_min=parser->scan_info.range_min;
    scanData.scan_time=parser->scan_info.scan_time;
    scanData.time_increment=parser->scan_info.time_increment;
    scanData.ranges.resize(parser->distance.size());
    scanData.intensities.resize(parser->distance.size()); 
    for(int i=0;i<parser->distance.size();i++)
    {      
        if(parser->intensity.empty())
        {
            if(parser->distance[i]>scanData.range_max*1000||parser->distance[i]<scanData.range_min*1000)
            {
                scanData.ranges[i]=0;
                scanData.intensities[i]=0;
            }
            else
            {
                scanData.ranges[i]=parser->distance[i]*0.001f;
                scanData.intensities[i]=0;
            }
        }
        else
        {
            if(parser->distance[i]>scanData.range_max*1000||parser->distance[i]<scanData.range_min*1000)
            {
                scanData.ranges[i]=0;
                scanData.intensities[i]=0;
            }
            else
            {
                scanData.ranges[i]=parser->distance[i]*0.001f;
                scanData.intensities[i]=parser->intensity[i];
            }
        }
    }
    int theory_size=parser->scan_info.num_size;
    int actual_size=scanData.ranges.size();
    //std:: cout<<theory_size<<"  "<< actual_size<<std::endl;
    if(actual_size==theory_size)
    {
        //std::cout<<"start connect6"<<std::endl;
    //std::cout<<scanData<<std::endl;
        scan_pub_.publish(scanData);
        
    }
    parser->clear_data();
}

void olelidar::Ros_node::SelectType(uint8_t* data)
{
     if(product_=="VF")
    {
        data_type=(int)*(data+11);
        //std::cout<<data_type<<std::endl;
        switch (data_type)
        {
        case 0:
            parser=std::unique_ptr<OLEPacket>(new VFPacket_A);  
            break;         
        case 1:
            parser=std::unique_ptr<OLEPacket>(new VFPacket_B);   
            break;        
        case 2:
            parser=std::unique_ptr<OLEPacket>(new VFPacket_C);            
            break;
        default:
            ROS_ERROR("No this datatype!");
            return;           
        }  
    }
}

olelidar::Ros_node::~Ros_node()
{
    is_loop=false;
    if(reader.joinable())
    {
        reader.join();
    }
    transport_->disconnect();
}
