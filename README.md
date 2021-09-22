# World Robotic Olympiad 2021 Future Engineers

## Introduction

The pandemic has accelerated the introduction of digital technologies in all spheres of life. 41% of company executives from 45 countries are preparing to work in a new post-crisis reality, investing in accelerating business automation.

Sustainable development of companies even before the pandemic was associated with digital technologies. Automation of transport gives many advantages, but it is also important that the introduction of machine labour reduces risks. Some of them are shortages of workers, low attractiveness of labour in the industry. These circumstances necessitate the use of labour-saving technologies: digital, intellectual and robotic. Basically, these technologies are represented by robotics, one of the most important functions of which is intellectual, it is to quickly obtain information, data on processes performed and products to improve the efficiency of decision-making and implementation of management decisions in autonomous movement. When introducing robotics in logistics organisations, it is necessary to take into account the regional characteristics and specifics of specific sectors of transportation. Regional features of robotics may be related to the peculiarities of the region's development. These features include the level and conditions of socio-economic development of the region, infrastructure development, demographic trends, competitiveness of the transport sector to attract labour in comparison with other industries, etc. Industry features that require robotisation may be related to the need to perform monotonous repetitive processes, the presence of heavy, dangerous and harmful activities. An important condition for robotisation of processes in the field of transportation is the possibility of compiling certain algorithms that underlie the functioning of robotics.

##### Based on the above facts, the following tasks of robotics in the field of transportation can be distinguished:

- Monitoring and forecasting;

- Reducing the cost of transportation;

- Improving logistics security;

- Solving personnel problems;

- Reducing the costs of employees' bad faith;

- Expanding the possibilities of using equipment - robots can be all-weather and work at any time of the day.


``Kinematic scheme``

``Calculation part``

>3.1 Engine calculation
    
1) Motor torque is the force with which it acts on the rotating axle. In order for the robot to move, it is necessary that this force exceeds the weight of the robot (expressed in N/m).

When travelling in a straight line at a distance of 1 m, calculate the acceleration required to achieve a speed of 1 m/s.

    V^2 = v_0^2+2ad

Where is the distance travelled by the robot - its initial speed (start from the place, so ),

    A = (v^2-v_0^2)/2d

Where is the speed of the robot, its acceleration.

Substitute the values adopted in our model, we'll get

    A = (1^2-0)/2=0.5 m/c^2 - acceleration required to achieve a speed of 1 m/s

The torque that is necessary to move the robot and obtain the acceleration necessary to achieve maximum speed is calculated as follows:

When - moment of inertia and - angular acceleration, we get

    M = mrag/2=(8*9.8*0.13*10)/2=51 
        (mN*m)=0.52 (kg*cm)

Here m/s2 - acceleration of free fall (round it to 10), - radius of the wheel, - mass of the entire robot

To convert the value expressed in Nm to kg·cm, it should be taken into account that 1H = 0.102 kg and 1m = 100 cm. Therefore, 50 mN·m = 50 · 0.102 : 1000 * 100 = 0.51 kg · cm.

2) To calculate the maximum power of the engines, we will need a speed that is expressed in revolutions per minute


    V=60ω/2πr (rpm)
#####
    V=(60*1)/(2π*0.13)=73.46 (rpm)

Motor power is proportional to torque and speed, substituting formulas for torque and frequency, we get:

    P=M*v=(8*9.8*0.13*0.5)/(2*0.13)=19.6 (W)

We got the total power for all engines, in our case the engines are two, so we need to divide the result by two.

For our two-wheeled robot, using a gear ratio equal to 10, the characteristics of the installed engines should be as follows:

**Speed** - 2000 rpm.

**Torque** - 60 mN*m

**Electric power** - 50 W

>3.2 Servo drive calculation

The robot on the turn of the front axle uses micro servo 9g sg 90 servo drives.

The torque on the shaft of this drive is 1.6 kg cm.

The maximum load of the inverted wheel is 1-1.3 kg.

When the servo drive is located on the axis of rotation of the swivel fist with a stop of the drive nozzle to the upper fist, we create a moment of force that allows you to keep the wheel in the required angle.

    M=[r *F ]

Where is M torque?

    F - power
    R - radius

Using this formula, we can find the power that the servo drive can withstand.

Given:

    M=1.6 (kg*cm) = 0.16 (H*m)
#####
    R=0.025( m)
#####
    F=M/r=0.16/0.025=6.4 (H)

Conclusion: Such power is enough for our task, as the servo acts through the lever, not directly.

>3.3 Screw calculation

-   Information system.

>>4.1 Electronic components 
>####4.1.1 selection of engine control driver.
>Dual L298N driver - made on the basis of the L298N chip with low internal resistance, PWM control capability and output current up to 5 A, which is a suitable solution for a project with high current consumption. The L298N driver is compatible with the entire Arduino line as well as with any other microcontrollers.

**Characteristics:**

- logic voltage 5 Volts;

- engine operating voltage from 7.2 to 20 Volts DC;

- operating output current to the channel up to 68A (start mode);

**Size:** 4.5*4.1 cm.

The motor paired driver works debugged, as the operating current is enough for the motor, which at its peak consumes 2A at 12V.

>4.1.2 Battery calculation

**Single battery:**

Formulas that determine the relationship between the current that the battery gives to the load, its capacity and the relative discharge rate:

    I_bat = C_rate*C_bat
#####
    I_bat - current in amperes given to the load by one battery;
    
**C_bat** - nominal battery capacity in ampere hours (means the product of amperes per watch), which is usually labelled on the battery;

C_rate is the relative discharge rate of the battery, defined as the discharge current divided by the theoretical current that the battery can give out within one hour and its capacity will be fully consumed.

**I_bat** = 30 A;

**C_bat** = 3000 mAh;

Where does **C_rate** = 10 seconds come from;

Operating time t and relative discharge rate of the C_rate battery are bound by an inverse proportional dependence:

    t=1/C_rate

Let's note that this is a theoretical time of work. Due to a variety of external factors, real operating time will be about 30% less than calculated according to this formula. It should also be noted that the permissible discharge depth of the battery further limits its operating time.

The watt-hour energy stored in the battery is calculated by the formula:

    E_bat=V_bat C_bat^

**Here:**

**E_bat** - nominal energy stored in the battery in watt hours,

**V_bat** - rated battery voltage in volts

**C_bat** - nominal battery capacity in ampere hours (Ah)

**V_bat** = 4.2 B;

**E_bat** = 12.6 Wh;

The energy in joules (watt seconds, Wt-s) is calculated by the formula

    E_(bat,joul) = 3600*E_(bat,Wh);
#####
    E_(bat,joul) = 45.36 kJ;

It is known that at a current of one ampere, a charge of one pendant passes through the cross section of the conductor in one second. Therefore, the battery charge is determined from the expression Q = I · t, taking into account the known battery capacity in ampere hours, which determines the current given by the battery to the load for 3600 seconds:

    Q_bat = 3600<OPUS-CC>*C_BAT_

**Q_bat** - battery charge in pendants (K)

Cbat is the nominal capacity of the battery in ampere hours.

**Q_bat** = 108 kCl;

**Battery pack**

The rated voltage of the battery pack in volts is determined by the formula:

V_bank=V_bat*N_s

**Here:**

**Vbat** - rated voltage of the battery in volts,

**Vbank** - rated voltage of the battery pack in volts

**Ns** - number of batteries in one of several groups of serially connected batteries;

**Ns** = 4 pcs.

**Vbank** = 16.8 V;

Battery capacity in ampere hours, **Cbank** is determined by the formula:

    C_bank=C_bat*N_p

**C_bank** = 12 Ah;

The nominal energy in Ebank watt hours stored in the battery pack is determined by the formula:

    E_bank = E_bat *N_p*N_s_

**Np** - the number of groups of serially connected batteries connected in parallel

**Np** = 4 pcs.

**E_bank** = 201.6 Wh

Joule energy is calculated by the formula:

    E_(bank,joul) = 3600*E_(bank,Wh)

Here Ebank, Wh is the nominal energy of the battery pack in watt hours.

    E_(bank,joul) = 725.76 kJ;

The charge in the pendants of the Qbank battery pack is defined as the sum of the charges of all batteries in the unit:

    Q_bank = Q_bat*N_p *N_s

**Q_bank**=1 728 kCl

The discharge current of the Ibank battery pack is calculated by the formula:

    I_bank = I_bat*N_p

**I_bank** = 120 A

The operating time of the tbank battery pack is determined by the formula:

    T_bank = t_bat *N_p

**T_bank** = 24 min.

4s2p battery assembled on 18650 high-current batteries. The choice was made on these batteries, as they have large capacity, low self-discharge, light and easy to maintain, and have a long cycle life.

Assembly of 8 high-current batteries, capacity of each 3000 mAh, assembly will be in 4S2P format (two 4 can batteries in parallel), which is supposed to give a total capacity of 12 Ah.

>4.2 Software algorithms

First of all, the software algorithm of the car is based on its mechanical structure.

**Software design:**

1. Reading the competition regulations

2. Finding permanent points in the image. In our case, I considered the sides a permanent point, as they are always black and are always in the area of operation of the camera.

3. Choosing a way to track signs. I decided to find the image mask according to the desired colour, find the outline and determine the middle of the sign.

4. Turn tracking algorithm. It's In this aspect, I was guided by field elements that can be seen at each new turn - colour lines. I cut out the mask in the same way and find the middle.

5. Harmonisation of the software and mechanical device of our car.

First of all, I read the regulations of the competition. Next, for the constant movement of the car, you need to choose beacons on which we can move during the attempt. It is also worth taking care of the detour of the signs by tracking them in the image. And I also solved the problem of counting the turns taken, counting the turning lanes. After writing the detailed code, I had to agree on the basic controls of the vehicle. This was realised in the precise rotation of the servo control of the front axle of the machine.

>4.2.1 schematic diagram
