// Renode peripheral: an ICM-42688-P seen only through four GPIO lines.
//
// On FCSTMV1.05 the IMU cannot use the F405's hardware SPI1 (PA6/PA7 are
// transposed, see docs/PINOUT.md), so the firmware bit-bangs SPI mode 3 on
// PA4..PA7. Renode's stock SPI models therefore never see a byte. This model
// sits on the GPIO lines instead and decodes the waveform the emulated MCU
// produces, the same way test/mocks/sim_spi_slave.hpp does on the host:
//
//   * CS falling starts a transaction, CS rising ends it
//   * on a falling SCK edge the next MISO bit is presented
//   * on a rising SCK edge MOSI is sampled, MSB first
//
// Inputs (wired in fcstmv105.repl):  0 = CS (PA4), 1 = SCK (PA5), 2 = SDI (PA6)
// Output:                            SDO -> PA7
//
// The register file holds a level, stationary airframe by default: accel
// (0, 0, -1) g and zero rate. Values can be changed from the monitor with
// SetAccelG / SetGyroDps. Everything here is a simulation, not silicon.

using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class Icm42688BitBang : IGPIOReceiver
    {
        public Icm42688BitBang(IGPIOReceiver sdoPort, int sdoPin = 7)
        {
            this.sdoPort = sdoPort;
            this.sdoPin = sdoPin;
            Reset();
        }

        // Observations used by the Robot test.
        public int Transactions { get; private set; }
        public int BurstReads { get; private set; }
        public int SoftResets { get; private set; }
        public int ProtocolErrors { get; private set; }

        public void Reset()
        {
            Array.Clear(regs, 0, regs.Length);
            regs[RegWhoAmI] = whoAmI;
            // Lines start at the GPIO port's reset level (low). A transaction
            // only begins on a falling CS edge, so the firmware raising CS and
            // SCK to idle during start-up is not mistaken for one.
            csHigh = false;
            sckHigh = false;
            mosiHigh = false;
            selected = false;
            bitIndex = 0;
            byteIndex = 0;
            shiftIn = 0;
            shiftOut = 0;
            Transactions = 0;
            BurstReads = 0;
            SoftResets = 0;
            ProtocolErrors = 0;
            sdoLevel = false;
            SetAccelG(0.0, 0.0, -1.0);
            SetGyroDps(0.0, 0.0, 0.0);
        }

        // Accel in g at the +/-16 g range the firmware configures.
        public void SetAccelG(double x, double y, double z)
        {
            accel[0] = x;
            accel[1] = y;
            accel[2] = z;
            LoadSamples();
        }

        // Rate in deg/s at the +/-2000 dps range the firmware configures.
        public void SetGyroDps(double x, double y, double z)
        {
            gyro[0] = x;
            gyro[1] = y;
            gyro[2] = z;
            LoadSamples();
        }

        // Identity byte. Changing it simulates a wrong or absent part.
        public void SetWhoAmI(byte value)
        {
            whoAmI = value;
            regs[RegWhoAmI] = value;
        }

        public byte ReadRegister(int address)
        {
            return regs[address & 0x7F];
        }

        public void OnGPIO(int number, bool value)
        {
            switch(number)
            {
                case 0:
                    OnChipSelect(value);
                    break;
                case 1:
                    OnClock(value);
                    break;
                case 2:
                    mosiHigh = value;
                    break;
                case 3:
                    // Feedback: the level the port now holds on the SDO pin.
                    if(value != sdoLevel)
                    {
                        DriveSdo();
                    }
                    break;
                default:
                    this.Log(LogLevel.Warning, "Unexpected GPIO input {0}", number);
                    break;
            }
        }

        // Renode 1.16.1's STM32_GPIOPort handles a BSRR write by snapshotting
        // every pin of the port and rewriting all sixteen, inputs included. When
        // the SCK edge in that write makes this model change SDO, the rewrite of
        // pin 7 that follows restores the stale level. The port's pin 7 output
        // is therefore wired back to input 3 and any level that disagrees with
        // what the device is driving is corrected on the spot. Upstream Renode
        // has since made the port mode aware, after which this path never fires.
        private void DriveSdo()
        {
            sdoPort.OnGPIO(sdoPin, sdoLevel);
        }

        private void OnChipSelect(bool high)
        {
            if(high == csHigh)
            {
                return;
            }
            csHigh = high;

            if(!high)
            {
                selected = true;
                ++Transactions;
                bitIndex = 0;
                byteIndex = 0;
                shiftIn = 0;
                shiftOut = 0;  // status byte during the address phase
                if(!sckHigh)
                {
                    // Mode 3 requires SCK parked high before CS falls.
                    ++ProtocolErrors;
                }
            }
            else
            {
                if(bitIndex != 0)
                {
                    // Transaction ended mid-byte.
                    ++ProtocolErrors;
                }
                selected = false;
            }
        }

        private void OnClock(bool high)
        {
            if(high == sckHigh)
            {
                return;
            }
            sckHigh = high;

            if(!selected)
            {
                return;
            }

            if(high)
            {
                if(mosiHigh)
                {
                    shiftIn = (byte)(shiftIn | (1 << (7 - bitIndex)));
                }
                ++bitIndex;
                if(bitIndex == 8)
                {
                    ProcessByte(shiftIn);
                    shiftIn = 0;
                    bitIndex = 0;
                    ++byteIndex;
                }
            }
            else
            {
                sdoLevel = ((shiftOut >> (7 - bitIndex)) & 1) != 0;
                DriveSdo();
            }
        }

        private void ProcessByte(byte b)
        {
            if(byteIndex == 0)
            {
                address = (byte)(b & 0x7F);
                isRead = (b & 0x80) != 0;
                if(isRead && address == RegTempData1)
                {
                    ++BurstReads;
                }
                shiftOut = isRead ? regs[address] : (byte)0;
                return;
            }

            if(!isRead)
            {
                WriteRegister(address, b);
            }
            address = (byte)((address + 1) & 0x7F);
            shiftOut = isRead ? regs[address] : (byte)0;
        }

        private void WriteRegister(byte addr, byte value)
        {
            if(addr == RegDeviceConfig && (value & 0x01) != 0)
            {
                ++SoftResets;
                Array.Clear(regs, 0, regs.Length);
                regs[RegWhoAmI] = whoAmI;
                LoadSamples();
                return;
            }
            regs[addr & 0x7F] = value;
        }

        private void LoadSamples()
        {
            for(var i = 0; i < 3; ++i)
            {
                SetBe16(RegAccelDataX1 + 2 * i, Scale(accel[i], AccelLsbPerG));
                SetBe16(RegGyroDataX1 + 2 * i, Scale(gyro[i], GyroLsbPerDps));
            }
            // 25 degC reads as zero in TEMP_DATA.
            SetBe16(RegTempData1, 0);
        }

        private static short Scale(double value, double lsbPerUnit)
        {
            var raw = Math.Round(value * lsbPerUnit);
            return (short)Math.Max(short.MinValue, Math.Min(short.MaxValue, raw));
        }

        private void SetBe16(int reg, short value)
        {
            regs[reg] = (byte)((value >> 8) & 0xFF);
            regs[reg + 1] = (byte)(value & 0xFF);
        }

        private readonly byte[] regs = new byte[128];
        private readonly double[] accel = new double[3];
        private readonly double[] gyro = new double[3];

        private readonly IGPIOReceiver sdoPort;
        private readonly int sdoPin;
        private bool sdoLevel;
        private byte whoAmI = WhoAmIValue;
        private bool csHigh;
        private bool sckHigh;
        private bool mosiHigh;
        private bool selected;
        private int bitIndex;
        private int byteIndex;
        private byte shiftIn;
        private byte shiftOut;
        private byte address;
        private bool isRead;

        private const int RegDeviceConfig = 0x11;
        private const int RegTempData1 = 0x1D;
        private const int RegAccelDataX1 = 0x1F;
        private const int RegGyroDataX1 = 0x25;
        private const int RegWhoAmI = 0x75;
        private const byte WhoAmIValue = 0x47;

        private const double AccelLsbPerG = 32768.0 / 16.0;
        private const double GyroLsbPerDps = 32768.0 / 2000.0;
    }
}
