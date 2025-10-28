using DeftSharp.Windows.Input.Mouse;
using InTheHand.Bluetooth;
using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Diagnostics;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using DeftSharp;
using static System.Windows.Forms.VisualStyles.VisualStyleElement;

namespace _49022AppRev2
{
    public partial class Form1 : Form
    {

        String cwOpt = "";
        String ccwOpt = "";
        String scOpt = "";
        String dcOpt = "";
        int mouseDir = 1;
        String dataTxt = "No Data Yet!";

        GattCharacteristic characteristic;
        BluetoothDevice connectedDevice;
        GattService selectedService;
        

        public Form1()
        {
            InitializeComponent();
        }
        private async void Form1_Load(object sender, EventArgs e)
        {
            //MessageBox.Show("Starting scan...");

            //try
            //{
            //    var devices = await Bluetooth.ScanForDevicesAsync();
            //    MessageBox.Show($"Scan complete. Found {devices.Count} devices.");

            //    if (devices.Count > 0)
            //    {
            //        string list = string.Join("\n", devices.Select(d => $"{d.Name ?? "(Unnamed)"} - {d.Id}"));
            //        MessageBox.Show(list);
            //    }
            //}
            //catch (Exception ex)
            //{
            //    MessageBox.Show("Scan failed: " + ex.Message);
            //}
            label5.Text = "Data Received: No Data Yet!";
            await ConnectToBLEDevice("Senior");
        }

        private async Task ConnectToBLEDevice(string deviceName)
        {
            try
            {
                MessageBox.Show("Looking for Devices");

                var devices = await Bluetooth.ScanForDevicesAsync();
                var device = devices.FirstOrDefault(d => d.Name == deviceName);


                if (device == null)
                {
                    MessageBox.Show("Device not found!");
                    return;
                }

                await device.Gatt.ConnectAsync();
                connectedDevice = device;

                MessageBox.Show($"Connected to {device.Name}");

                // Discover services
                MessageBox.Show("Finding Services");
                var services = await device.Gatt.GetPrimaryServicesAsync();
                selectedService = services.FirstOrDefault(s => s.Uuid.ToString() == "99887766-5544-3322-1100-ffeeddccbbaa");

                if (selectedService != null)
                {
                    MessageBox.Show("Service found!");
                    var characteristics = await selectedService.GetCharacteristicsAsync();
                    characteristic = characteristics.FirstOrDefault();

                    foreach (var ch in characteristics)
                    {
                        MessageBox.Show($"{ch.Uuid} -> {ch.Properties}");
                    }

                    if (characteristic != null)
                    {
                        // Subscribe to notifications (data received)
                        characteristic.CharacteristicValueChanged += Characteristic_CharacteristicValueChanged;
                        await characteristic.StartNotificationsAsync();
                        MessageBox.Show("Subscribed to characteristic notifications.");
                    }
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"BLE connection error:\n{ex}\n\nStack:\n{ex.StackTrace}");
            }
        }

        private void Characteristic_CharacteristicValueChanged(object sender, GattCharacteristicValueChangedEventArgs e)
        {
            // Convert received bytes to string (or parse as needed)
            var data = Encoding.UTF8.GetString(e.Value);
            var mouse = new MouseManipulator();
            var mouseListener = new MouseListener();
            var position = mouseListener.Position;
            dataTxt = data;
            this.Invoke(new Action(() =>
            {
                label5.Text = $"Data Received: {data}";
            }));
            if (data == "Hello")
            {
                position = mouseListener.Position;
                mouse.SetPosition(position.X + mouseDir * 400, position.Y + mouseDir * 250);
                mouseDir = -1 * mouseDir;
            }
        }

        private void button1_Click(object sender, EventArgs e)
        {
            cwOpt = comboBox1.SelectedItem.ToString();
            ccwOpt = comboBox2.SelectedItem.ToString();
            scOpt = comboBox3.SelectedItem.ToString();
            dcOpt = comboBox4.SelectedItem.ToString();
        }

        private void comboBox2_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void comboBox1_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void comboBox3_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void comboBox4_SelectedIndexChanged(object sender, EventArgs e)
        {

        }

        private void textBox1_TextChanged(object sender, EventArgs e)
        {

        }

        private void textBox2_TextChanged(object sender, EventArgs e)
        {

        }

        private void label1_Click(object sender, EventArgs e)
        {

        }

        private void label4_Click(object sender, EventArgs e)
        {

        }

        private void label2_Click(object sender, EventArgs e)
        {

        }

        private void label5_Click(object sender, EventArgs e)
        {

        }
    }
}
