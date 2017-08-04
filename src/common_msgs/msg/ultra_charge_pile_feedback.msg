uint8    work_status            #工作状态(按位) 0：上电无电流 1：电流超载 2：充电电压过高 3：电压过低
uint8    charge_status          #充电状态，0-未充电 1-涓流充电 2-正常充电中 3-浮充 4-充电完成
uint8    limit_switch           #限位开关 0-无触碰 1-已触碰
uint16   voltage_status         #电压状态mv，0-200对应10.0v-30.0v (计算方法:(100+index)*100)mv
uint16   current_status         #电流状态mA,0-100对应 0A-10.0A (计算方法:index*100)mA
