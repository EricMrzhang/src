#include <common/public.h>
#include <iostream>
#include <fstream>
#include <sstream>
//可以设置位也可以清除位
void SetBit(unsigned int &value, int bit, int bit_v)
{
    if(bit_v)  value|=1<<bit; //仅在 bit 索引位置为1，其余位为0的数
    //生成一个只在指定位上有1的掩码，
    //但随后通过按位取反操作（~）生成一个在该位上为0，其余位为1的掩码。
    else value&=~(1<<bit);  
}
//设置或清除 value 中的特定
void SetErr(unsigned int &value, int bit, int bit_v)
{
    // printf("zhengchang=%d\n",bit_v);
    if(bit_v)  value=0;
    else value=bit;
}

// void SetNodeErr()



vector<string> getFileList(char *basePath,char *sep)
{
    DIR *dir;
    struct dirent *ptr;
    vector<string> filelist;

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
    }

    filelist.clear();
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            vector<string> ss=split(ptr->d_name,".");
            if(ss.size()>1 && ss[ss.size()-1]==sep) filelist.push_back(ss[0]);
        }
        else if(ptr->d_type == 10)   //link file
        {
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
        }
        else if(ptr->d_type == 4)    ///dir
        {
        }
    }
    closedir(dir);

    return filelist;
};

void GetPackagePath(char *packname,char *path)
{
    char cmd[100];
    sprintf(cmd,"rospack find %s",packname);
    FILE *fp=popen(cmd,"r");

    fgets(path, 256, fp);
    pclose(fp);
    //printf("%s\n",path);
}

vector<string> split(const string& s, const string& sep)
{
    vector<string> v;
    string::size_type pos1, pos2;
    pos2 = s.find(sep);
    pos1 = 0;
    while(string::npos != pos2)
    {
        v.push_back(s.substr(pos1, pos2-pos1));

        pos1 = pos2 + sep.size();
        pos2 = s.find(sep, pos1);
    }
    if(pos1 != s.length())
        v.push_back(s.substr(pos1));
    return v;
};


LS_Coeff Least_Square(vector<geometry_msgs::Point> point)
{
    LS_Coeff A={0,0,0};

    if(point.size()==2) 
    {
        A.a1=(point[1].y-point[0].y)/(point[1].x-point[0].x);
        A.a0=point[1].y-A.a1*point[1].x;
        return A;
    }

    double sumX = 0, sumY = 0, sumXX = 0, sumXXX = 0, sumXXXX = 0, sumXY = 0, sumXXY = 0;
    for(int i=0;i<point.size();i++)
    {
        sumX=sumX+point.at(i).x;
        sumY=sumY+point.at(i).y;
        sumXX=sumXX+point.at(i).x*point.at(i).x;
        sumXXX=sumXXX+point.at(i).x*point.at(i).x*point.at(i).x;
        sumXXXX=sumXXXX+point.at(i).x*point.at(i).x*point.at(i).x*point.at(i).x;
        sumXY=sumXY+point.at(i).x*point.at(i).y;
        sumXXY=sumXXY+point.at(i).x*point.at(i).x*point.at(i).y;
    }
    double a11 = point.size();   double  a12 = sumX;   double  a13 = sumXX;   double  b1 = sumY;
    double  a21 = sumX;          double  a22 = sumXX;  double  a23 = sumXXX;  double  b2 = sumXY;
    double  a31 = sumXX;         double  a32 = sumXXX; double  a33 = sumXXXX; double  b3 = sumXXY;

    double a21x = a21 - a21 / a11*a11;
    double a22x = a22 - a21 / a11*a12;
    double a23x = a23 - a21 / a11*a13;
    double b2x = b2 - a21 / a11*b1;

    double a31x = a31 - a31 / a11*a11;
    double  a32x = a32 - a31 / a11*a12;
    double  a33x = a33 - a31 / a11*a13;
    double b3x = b3 - a31 / a11*b1;

    double a32xx = a32x - a32x / a22x*a22x;
    double  a33xx = a33x - a32x / a22x*a23x;
    double  b3xx = b3x - a32x / a22x*b2x;

    A.a2=b3xx / a33xx;
    A.a1=(b2x - a23x*A.a2) / a22x;
    A.a0=(b1 - A.a1*a12 - A.a2*a13) / a11;
    return A;
};

vector<float> FittingCurve(vector<float> XData, vector<float> YData, int m)
{
    int i,j,k,n;
    float z,p,c,g,q,d1,d2,h;
    float a[20],s[20],t[20],b[20];
    vector<float> coeff;

    n = XData.size();
    if(n<m) m=n;
    for(i=0; i<m; i++) a[i]=0.0;
    z=0.0;
    for(i=0; i<n; i++) z+=XData[i]/n;
    // printf("%.2f\n",z);
    b[0]=1.0;  d1=n;  p=0.0;  c=0.0;
    for(i=0; i<n; i++)
    {
       p+=(XData[i]-z);
       c+=YData[i];
    }
    c/=d1;  p/=d1;
    a[0]=c*b[0];

    if(m>1)
    {
       t[1]=1.0;  t[0]=-p;
       d2=0.0;  c=0.0;  g=0.0;
       for(i=0; i<n; i++)
       {
           q=XData[i]-z-p;   d2=d2+q*q;
           c+=YData[i]*q;
           g+=(XData[i]-z)*q*q;
       };
       c/=d2; p=g/d2; q=d2/d1;
       d1=d2;
       a[1]=c*t[1];  a[0]=c*t[0]+a[0];
    }

    for(j=2; j<m; j++)
    {
       s[j]=t[j-1];
       s[j-1]=-p*t[j-1]+t[j-2];
       if(j>=3)
           for(k=j-2; j>=1; j--) s[k]=-p*t[k]+t[k-1]-q*b[k];
       s[0]=-p*t[0]-q*b[0];

       d2=c=g=0.0;
       for(i=0; i<n; i++)
       {
          q=s[j];
          for(k=j-1; k>=0; k--) q=q*(XData[i]-z)+s[k];
          d2=d2+q*q;  c=c+YData[i]*q;
          g=g+(XData[i]-z)*q*q;
       }
       c=c/d2;  p=g/d2;  q=d2/d1;
       d1=d2;
       a[j]=c*s[j];  t[j]=s[j];
       for(k=j-1; k>=0; k--)
       {
          a[k]=c*s[k]+a[k];
          b[k]=t[k];  t[k]=s[k];
       }
    }

    for(i=0; i<m; i++) coeff.push_back(a[i]);
    return coeff;
    
}

vector<string> readFileList(char *basePath)
{
    DIR *dir;
    struct dirent *ptr;
    vector<string> filelist;

    if ((dir=opendir(basePath)) == NULL)
    {
        perror("Open dir error...");
    }

    filelist.clear();
    while ((ptr=readdir(dir)) != NULL)
    {
        if(strcmp(ptr->d_name,".")==0 || strcmp(ptr->d_name,"..")==0)    ///current dir OR parrent dir
            continue;
        else if(ptr->d_type == 8)    ///file
        {
            filelist.push_back(ptr->d_name);
        }
        else if(ptr->d_type == 10)    ///link file
        {
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
        }
        else if(ptr->d_type == 4)    ///dir
        {
        }
    }
    closedir(dir);

    return filelist;
}

int FindMinID(vector<float> datalist)
{
    if(datalist.size()==0) return -1;

    vector<float>::iterator itMin = min_element(datalist.begin(), datalist.end());
    return distance(datalist.begin(), itMin);
}

//计算两点 (x1, y1) 和 (x2, y2) 之间的欧几里得距离
double P2P(double x1, double y1, double x2, double y2)
{
    double r = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
    if (r > 0.01)
        r = sqrt(r);
    else
        r = 0;

    return r;
}

float P2P(geometry_msgs::Point p1, geometry_msgs::Point p2)
{
    float r;
    //长度？
    r = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    r = sqrt(r);
    return r;
}

// 计算给定数据（ptr指向的字节数组，长度为len）的16位循环冗余校验（CRC）值的
uint16_t CRC16(uint8_t *ptr, uint32_t len)
{
    uint16_t crc = 0xffff;  //初始化CRC值
    uint32_t i;
    uint8_t ch;

    for (i = 0; i < len; i++)
    {
        //从ptr指向的地址读取一个字节到变量ch中，并将ptr指针向前移动一个字节的位置，
        ch = *ptr++;
        //计算ch和当前crc值的异或结果，并将这个结果的低4位用作查找表crctalbeabs的索引
        //将查找表返回的值与crc值右移4位的结果进行异或操作
        crc = crctalbeabs[(ch ^ crc) & 15] ^ (crc >> 4);

        crc = crctalbeabs[((ch >> 4) ^ crc) & 15] ^ (crc >> 4);
    }

    return crc;
}

//将当前时间转换为字符型格式： 格式化年月日时分秒
string NowtimetoString()
{   
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();    
    // 毫秒数                                               
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000; 
     // 转为 time_t 类型
    time_t t = std::chrono::system_clock::to_time_t(now);                                     
    tm tm_now;
    localtime_r(&t, &tm_now); // 转为本地时区的 tm 结构体
    stringstream ss;
    ss << std::put_time(&tm_now, "%Y-%m-%d-%H-%M-%S"); // 格式化年月日时分秒
    // ss << std::setfill('0') << std::setw(3) << ms.count(); // 格式化毫秒数，不足三位前面补零
    return ss.str();
}

TTimer::TTimer()
{
    Clear();
    value = 0;
}

void TTimer::Clear(float t)
{
    gettimeofday(&tv1, &tz);  //用于获取当前的时间（包括秒和微秒）
    tv2 = tv1;
    t_off = t;
}

//计算时间
double TTimer::GetValue(void)
{
    double ret;
    gettimeofday(&tv2, &tz);  //获取当前精确时间
    //计算时间
    ret = (tv2.tv_sec - tv1.tv_sec) + (tv2.tv_usec - tv1.tv_usec) * 0.000001 + t_off;
    value = ret;
    return ret;
}


THeartbeat::THeartbeat(float dt)
{
    interval=dt;
    start();   
}

void THeartbeat::Beat(int v)
{
    mycounter+=v;
}

void THeartbeat::run()
{
    TTimer tmr;
    while(1)
    {
        if (tmr.GetValue() > interval)   //判断是否超过interval时间
        {
            value = mycounter * 1.0 / tmr.GetValue();
            mycounter = 0;
            tmr.Clear();
        }
        usleep(20000);
    }
}

//赋值
THeartbeat_::THeartbeat_(ros::NodeHandle* n, string c, float dt)
{
    interval=dt;
    nh=n;
    caption=c;
    start();
}


//
void THeartbeat_::Beat(int v)
{
    mycounter+=v;
    // printf("AAAAAAAAAAAAAAAAAAAAAAAA %d\n", mycounter);
    // if(caption=="wsk1")  printf("%s %d\n", caption.c_str(), mycounter);
}

//设置最大值、最小值
void THeartbeat_::SetLimit(float min, float max)
{
    min_value=min,  max_value=max;
}

void THeartbeat_::run()
{
    TTimer tmr;
    value = 0;/////////////////////
    while(ros::ok())
    {
        // if(caption=="wsk1") printf("timer is running!\n");
        if(tmr.GetValue()>interval)
        {
            // if(caption=="wsk") printf("A %d %.4f\n", mycounter, tmr.GetValue());
            value=mycounter*1.0/tmr.GetValue();  
            // if(caption=="wsk1") printf("B %d %.1f %.4f\n", mycounter, value, tmr.GetValue());
            mycounter=0;
            tmr.Clear();  //时间清零

            char buf[100]={0};
            //小于最小值
            if(value<min_value)  sprintf(buf,"%s %.1f is lower than %.1f", caption.c_str(), value, min_value);
            else if(value>max_value)  sprintf(buf,"%s %.1f is higher than %.1f", caption.c_str(), value, max_value);
            err_msg=buf;  //保存缓冲器中数据
            if(err_msg=="") ok_flag=true;
            else ok_flag=false;

            if(nh!=NULL) 
            {
                nh->setParam("check/"+caption, value);     //发布参数
            }
        }

        usleep(20000);
    }
}

TNodeCheck::TNodeCheck(ros::NodeHandle *n, string caps, float dt)
{
    nh=n;
    vector<string> strs=split(caps," ");  //拆分
    check_dt=dt;
    // printf("%d\n", strs.size());
    THeartbeat_ *bt;
    for(auto it:strs)
    {   
        //创建了一个新的 THeartbeat_ 对象，
        //使用给定的参数调用其构造函数，并将返回的指针赋值给变量 bt
        bt=new THeartbeat_(nh,it,dt);
        heartbeat_array.push_back(bt);
    }

    usleep(50000);
    start();
}

//在heartbeat_array中查找第一个其caption成员与给定参数相等的THeartbeat_对象，
//并返回指向该对象的指
THeartbeat_* TNodeCheck::Find(string caption)
{
    THeartbeat_* res=NULL;
    for(int i=0;i<heartbeat_array.size();i++)
        //检查当前遍历到的heartbeat_array中的元素所指向的对象的caption成员是否等于函数参数caption
        if(heartbeat_array[i]->caption==caption)
        {
            res=heartbeat_array[i];
            break;
        }
    return res;
}

void TNodeCheck::run()
{
    TTimer tmr;
    while (ros::ok())
    {
        // continue;
        if(tmr.GetValue()>check_dt)  //检查时间
        {
            tmr.Clear();  //时间清零


            string check_msg="";
            for(auto it:heartbeat_array)
            {
                if(it->err_msg!="" && check_msg=="")  check_msg=it->err_msg;
                else if(it->err_msg!="" && check_msg!="")
                {
                    check_msg+=";",  check_msg+=it->err_msg;
                }
            }

            if(check_msg=="")  
            {
                char buf[200];
                sprintf(buf,"OK, seq %d",seq);  //格式化保存seq于buf中
                check_msg=buf;
            }    
            
            nh->setParam("check/msg", check_msg);   //发布参数
            // nh->setParam("check/seq", seq);         //发布参数
        }

        seq++;
        if(seq>1000) seq=0;
        nh->setParam("check/seq", seq);         //发布参数
        usleep(10000);
    }
}

//赋值
TDataFilter::TDataFilter(int n)
{
    filternum = n;
}

double TDataFilter::GetValue(double v)
{
    buf.push_back(v);  //尾加
    if (buf.size() > filternum)
        buf.erase(buf.begin());  //删除第一个元素

    value = 0;
    for (int i = 0; i < buf.size(); i++)
        value += buf[i];

    value /= buf.size();    //平均数
    return value;
}

//清除或重置缓冲区中的内容
void TDataFilter::Clear()
{
    buf.clear();
}

//执行一个ping命令，将结果重定向到一个临时文件，
//然后尝试打开这个文件以进行进一步的处理
//计算响应时间
int getPingTime(const std::string& ip)
{
    std::ostringstream commandStream;
    //构建命令字符串
    commandStream << "ping -c 1 " << ip << " > ping_tmp.txt";
    //commandStream对象中累积的字符串内容作为std::string对象返回。
    std::system(commandStream.str().c_str());
    //以输入模式打开一个名为ping_tmp.txt的文件，并将文件流对象赋给file
    std::ifstream file("ping_tmp.txt");
    if (!file.good())   //检查文件流对象的状态是否良好
    {
        std::cerr << "Failed to open ping result file" << std::endl;
        return 0;
    }

    std::string line;
    //逐行读取文件：
    while (std::getline(file, line)) {
        if (line.find("时间=") != std::string::npos) 
        {
            int startIndex = line.find("时间=") + 7;
            int endIndex = line.find(" 毫秒", startIndex);
            // printf("%s\n", line.substr(startIndex, endIndex - startIndex).c_str());
            // std::cout << startIndex << " " << endIndex << std::endl;
            //计算响应时间
            return std::stoi(line.substr(startIndex, endIndex - startIndex));
        }
    }

    return -1;
}


