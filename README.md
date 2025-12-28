# XC Tracker files

**xc_direct_esp32_receiver** uses an esp for compute, and an nrf52840 that reads the radio packets and sends them via uart
**xc_direct_radio_test_rx**  is the receiver 
**xc_direct_radio_test_tx** is  the transmitter
**xc_patriot_logger_nrf52840** is a try using ONLY the nrf52840, but i realized you cant use the radio and BLE, and since it also has no wifi its not a way forward
**xc_wifi_tracker_board_tests** Tests usingthe wifi tracker board-- really just getting the gps module working and printong on the screen
