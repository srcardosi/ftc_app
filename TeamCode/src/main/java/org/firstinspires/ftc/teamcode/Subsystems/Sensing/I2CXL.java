package org.firstinspires.ftc.teamcode.Subsystems.Sensing;

/**
 * Created by Evan McLoughlin on 12/18/2017.
 */

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import java.util.HashMap;


@I2cSensor(name = "MaxSonar I2CXL v2", description = "MaxSonar I2CXL Sensor from MaxBotix", xmlTag = "MaxSonarI2CXLv2")
public class I2CXL extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    public boolean error = false;
    public int lastDistance = -1;
    private long lastPingTime;

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.ModernRobotics;
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        return true;
    }

    @Override
    public String getDeviceName()
    {
        return "MaxSonarI2CXLv2";
    }

    public I2CXL(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.deviceClient.setI2cAddress(I2cAddr.create8bit(0xE0));

        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    protected I2CXL(I2cDeviceSynch i2cDeviceSynch, boolean deviceClientIsOwned)
    {
        super(i2cDeviceSynch, deviceClientIsOwned);
    }

    public String connectionInfo(){
        return super.getConnectionInfo();
    }

    public boolean getError(){
        return error;
    }

    public void setI2cAddress(I2cAddr i2cAddr)
    {
        deviceClient.setI2cAddress(i2cAddr);
    }

    private void ping()
    {
        deviceClient.write8(0, 0x51, I2cWaitControl.WRITTEN);
    }

    public int getDistance()
    {
        long curTime;
        boolean waitingForNextPing = true;

        while (waitingForNextPing) {
            curTime = System.currentTimeMillis();
            if((curTime - lastPingTime) > 100) {
                ping();
                lastPingTime = System.currentTimeMillis();
                waitingForNextPing = false;
            }
        }

        while (!waitingForNextPing) {
            curTime = System.currentTimeMillis();
            if((curTime - lastPingTime) > 80){
                int potentialDistance = TypeConversion.byteArrayToShort(deviceClient.read(0x01, 2));

                if(potentialDistance>1){
                    error = false;
                    lastDistance = potentialDistance;
                }
                else{
                    error = true;
                }

                waitingForNextPing = true;
            }
        }

        return lastDistance;
    }

    public static int mode(int []array)
    {
        HashMap<Integer,Integer> hm = new HashMap<Integer,Integer>();
        int max  = 1;
        int temp = 0;

        for(int i = 0; i < array.length; i++) {

            if (hm.get(array[i]) != null) {

                int count = hm.get(array[i]);
                count++;
                hm.put(array[i], count);

                if(count > max) {
                    max  = count;
                    temp = array[i];
                }
            }

            else
                hm.put(array[i],1);
        }
        return temp;
    }

    public int sampleDistance(){

        //declare the array
        int distanceArray[];

        //allocate memory for 10 indices
        distanceArray = new int[10];

        for (int loopCount = 0; loopCount < distanceArray.length; loopCount++ ) {
            distanceArray[loopCount] = getDistance();
        }

        int distance = mode(distanceArray);

        return distance;


    }

    public int sampleDistance100(){

        //declare the array
        int distanceArray[];

        //allocate memory for 100 indices
        distanceArray = new int[100];

        for (int loopCount = 0; loopCount < distanceArray.length; loopCount++ ) {
            distanceArray[loopCount] = getDistance();
        }

        int distance = mode(distanceArray);

        return distance;


    }
}