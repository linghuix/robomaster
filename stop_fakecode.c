


int min = 可接受最小距离
int max = 可接受最大距离
int trigger = 触发距离(如果要自动的话)
int diff = 平行状态两侧距离之差

//默认distance[0]在左边 PA4 接的那个口

void Stop_to_Supply(void){
    
    if(distance[0]<trigger && distance[1]<trigger){

        while(abs(distance[0]-distance[1])>= diff ){ //这里是先调整对齐，大概平行

           else if(distance[0] < distance[1]) 
            TURN_LEFT_SHORTLY;
           else if(distance[0] > distance[1]) 
            TURN_RIGHT_SHORTLY;

        }

        if(distance[0]>min && distance[0]<max && distance[1]>min && distance[1]<max)  //距离合适停车
            STOP;
        else if(distance[0]>max && distance[1]>max) //距离没到 正常向前
            DRIVE;
        else if(distance[0]<min && distance[1]<min) //太近 向后给个短时间的速度
            BACK_SHORTLY;
        else if(distance[0]<min && distance[1]>max) //左偏右偏看传感器放的位置
            TURN_LEFT_SHORTLY;
        else if(distance[0]>max && distance[1]<min) //左偏右偏看传感器放的位置
            TURN_RIGHT_SHORTLY;
    }
}
