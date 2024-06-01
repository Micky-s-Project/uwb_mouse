att_dispatch_client_can_send_now = 0x00005799;
att_dispatch_client_request_can_send_now_event = 0x0000579f;
att_dispatch_register_client = 0x000057a5;
att_dispatch_register_server = 0x000057b9;
att_dispatch_server_can_send_now = 0x000057cd;
att_dispatch_server_request_can_send_now_event = 0x000057d3;
att_emit_general_event = 0x00005885;
att_server_can_send_packet_now = 0x00005fb1;
att_server_deferred_read_response = 0x00005fb5;
att_server_get_mtu = 0x00005fcd;
att_server_indicate = 0x00006045;
att_server_init = 0x000060c9;
att_server_notify = 0x00006105;
att_server_register_packet_handler = 0x0000621d;
att_server_request_can_send_now_event = 0x0000624f;
att_set_db = 0x0000626b;
att_set_read_callback = 0x0000627d;
att_set_write_callback = 0x00006289;
bd_addr_cmp = 0x000063f9;
bd_addr_copy = 0x000063ff;
bd_addr_to_str = 0x00006409;
big_endian_read_16 = 0x00006441;
big_endian_read_32 = 0x00006449;
big_endian_store_16 = 0x0000645d;
big_endian_store_32 = 0x00006469;
btstack_config = 0x000065bd;
btstack_get_capabilities = 0x000065c9;
btstack_memory_pool_create = 0x00006711;
btstack_memory_pool_free = 0x00006739;
btstack_memory_pool_get = 0x00006799;
btstack_push_user_msg = 0x00006801;
btstack_push_user_runnable = 0x0000680d;
btstack_reset = 0x00006819;
char_for_nibble = 0x00006afd;
eTaskConfirmSleepModeStatus = 0x00006de1;
gap_add_dev_to_periodic_list = 0x000073c9;
gap_add_whitelist = 0x000073d9;
gap_aes_encrypt = 0x000073e5;
gap_clear_white_lists = 0x0000741d;
gap_clr_adv_set = 0x00007429;
gap_clr_periodic_adv_list = 0x00007435;
gap_create_connection_cancel = 0x00007441;
gap_disconnect = 0x0000744d;
gap_disconnect2 = 0x00007455;
gap_disconnect_all = 0x00007481;
gap_ext_create_connection = 0x000074c1;
gap_get_connection_parameter_range = 0x00007599;
gap_le_read_channel_map = 0x000075d1;
gap_periodic_adv_create_sync = 0x00007631;
gap_periodic_adv_create_sync_cancel = 0x00007655;
gap_periodic_adv_term_sync = 0x00007661;
gap_read_periodic_adv_list_size = 0x000076e9;
gap_read_phy = 0x000076f5;
gap_read_remote_used_features = 0x00007701;
gap_read_remote_version = 0x0000770d;
gap_read_rssi = 0x00007719;
gap_read_white_lists_size = 0x00007725;
gap_remove_whitelist = 0x00007731;
gap_rmv_adv_set = 0x000077ad;
gap_rmv_dev_from_periodic_list = 0x000077b9;
gap_rx_test_v2 = 0x000077c9;
gap_set_adv_set_random_addr = 0x00007801;
gap_set_callback_for_next_hci = 0x00007825;
gap_set_connection_parameter_range = 0x00007845;
gap_set_data_length = 0x0000785d;
gap_set_def_phy = 0x00007875;
gap_set_ext_adv_data = 0x00007885;
gap_set_ext_adv_enable = 0x0000789d;
gap_set_ext_adv_para = 0x0000790d;
gap_set_ext_scan_enable = 0x000079d5;
gap_set_ext_scan_para = 0x000079ed;
gap_set_ext_scan_response_data = 0x00007a8d;
gap_set_host_channel_classification = 0x00007aa5;
gap_set_periodic_adv_data = 0x00007ab5;
gap_set_periodic_adv_enable = 0x00007b25;
gap_set_periodic_adv_para = 0x00007b35;
gap_set_phy = 0x00007b4d;
gap_set_random_device_address = 0x00007b69;
gap_start_ccm = 0x00007b85;
gap_test_end = 0x00007bcd;
gap_tx_test_v2 = 0x00007bd9;
gap_tx_test_v4 = 0x00007bf1;
gap_update_connection_parameters = 0x00007c15;
gap_vendor_tx_continuous_wave = 0x00007c59;
gatt_client_cancel_write = 0x00008181;
gatt_client_discover_characteristic_descriptors = 0x000081a7;
gatt_client_discover_characteristics_for_handle_range_by_uuid128 = 0x000081e7;
gatt_client_discover_characteristics_for_handle_range_by_uuid16 = 0x00008237;
gatt_client_discover_characteristics_for_service = 0x00008287;
gatt_client_discover_primary_services = 0x000082bd;
gatt_client_discover_primary_services_by_uuid128 = 0x000082ef;
gatt_client_discover_primary_services_by_uuid16 = 0x00008333;
gatt_client_execute_write = 0x0000836f;
gatt_client_find_included_services_for_service = 0x00008395;
gatt_client_get_mtu = 0x000083c3;
gatt_client_is_ready = 0x0000846d;
gatt_client_listen_for_characteristic_value_updates = 0x00008483;
gatt_client_prepare_write = 0x000084a5;
gatt_client_read_characteristic_descriptor_using_descriptor_handle = 0x000084e1;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle = 0x0000850b;
gatt_client_read_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008511;
gatt_client_read_long_value_of_characteristic_using_value_handle = 0x0000853f;
gatt_client_read_long_value_of_characteristic_using_value_handle_with_offset = 0x00008545;
gatt_client_read_multiple_characteristic_values = 0x00008573;
gatt_client_read_value_of_characteristic_using_value_handle = 0x000085a3;
gatt_client_read_value_of_characteristics_by_uuid128 = 0x000085d1;
gatt_client_read_value_of_characteristics_by_uuid16 = 0x0000861d;
gatt_client_register_handler = 0x00008669;
gatt_client_reliable_write_long_value_of_characteristic = 0x00008675;
gatt_client_signed_write_without_response = 0x00008aa5;
gatt_client_write_characteristic_descriptor_using_descriptor_handle = 0x00008b69;
gatt_client_write_client_characteristic_configuration = 0x00008ba3;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle = 0x00008bf5;
gatt_client_write_long_characteristic_descriptor_using_descriptor_handle_with_offset = 0x00008c05;
gatt_client_write_long_value_of_characteristic = 0x00008c41;
gatt_client_write_long_value_of_characteristic_with_offset = 0x00008c51;
gatt_client_write_value_of_characteristic = 0x00008c8d;
gatt_client_write_value_of_characteristic_without_response = 0x00008cc3;
hci_add_event_handler = 0x0000a1e1;
hci_power_control = 0x0000a979;
hci_register_acl_packet_handler = 0x0000ab2d;
kv_commit = 0x0000b291;
kv_get = 0x0000b2ed;
kv_init = 0x0000b2f9;
kv_init_backend = 0x0000b379;
kv_put = 0x0000b38d;
kv_remove = 0x0000b399;
kv_remove_all = 0x0000b3cd;
kv_value_modified = 0x0000b3fd;
kv_value_modified_of_key = 0x0000b419;
kv_visit = 0x0000b425;
l2cap_add_event_handler = 0x0000b4b5;
l2cap_can_send_packet_now = 0x0000b4c5;
l2cap_create_le_credit_based_connection_request = 0x0000b681;
l2cap_credit_based_send = 0x0000b7c5;
l2cap_credit_based_send_continue = 0x0000b7f1;
l2cap_disconnect = 0x0000b86d;
l2cap_get_le_credit_based_connection_credits = 0x0000babd;
l2cap_get_peer_mtu_for_local_cid = 0x0000bad9;
l2cap_init = 0x0000bead;
l2cap_le_send_flow_control_credit = 0x0000bfa3;
l2cap_max_le_mtu = 0x0000c2ad;
l2cap_register_packet_handler = 0x0000c3d5;
l2cap_register_service = 0x0000c3e1;
l2cap_request_can_send_now_event = 0x0000c4f1;
l2cap_request_connection_parameter_update = 0x0000c50b;
l2cap_send_echo_request = 0x0000c9e5;
l2cap_unregister_service = 0x0000caa5;
le_device_db_add = 0x0000cafd;
le_device_db_find = 0x0000cbd5;
le_device_db_from_key = 0x0000cc01;
le_device_db_iter_cur = 0x0000cc09;
le_device_db_iter_cur_key = 0x0000cc0d;
le_device_db_iter_init = 0x0000cc11;
le_device_db_iter_next = 0x0000cc19;
le_device_db_remove_key = 0x0000cc3f;
ll_aes_encrypt = 0x0000cc6d;
ll_config = 0x0000cce9;
ll_free = 0x0000cd1f;
ll_get_capabilities = 0x0000cd29;
ll_get_heap_free_size = 0x0000cd49;
ll_get_states = 0x0000cd59;
ll_hint_on_ce_len = 0x0000ce15;
ll_legacy_adv_set_interval = 0x0000ce4d;
ll_malloc = 0x0000ce5d;
ll_query_timing_info = 0x0000cf95;
ll_register_hci_acl_previewer = 0x0000cfe1;
ll_scan_set_fixed_channel = 0x0000d045;
ll_set_adv_access_address = 0x0000d25d;
ll_set_adv_coded_scheme = 0x0000d269;
ll_set_conn_acl_report_latency = 0x0000d299;
ll_set_conn_coded_scheme = 0x0000d2c9;
ll_set_conn_latency = 0x0000d2f5;
ll_set_conn_tx_power = 0x0000d325;
ll_set_def_antenna = 0x0000d36d;
ll_set_initiating_coded_scheme = 0x0000d389;
ll_set_max_conn_number = 0x0000d395;
nibble_for_char = 0x0001d81d;
platform_calibrate_rt_clk = 0x0001d8bb;
platform_call_on_stack = 0x00004183;
platform_cancel_us_timer = 0x0001d8bf;
platform_config = 0x0001d8d5;
platform_create_us_timer = 0x0001d9f9;
platform_delete_timer = 0x0001da0d;
platform_enable_irq = 0x0001da15;
platform_get_current_task = 0x0001da4d;
platform_get_gen_os_driver = 0x0001da71;
platform_get_heap_status = 0x0001da79;
platform_get_link_layer_interf = 0x0001da91;
platform_get_task_handle = 0x0001da99;
platform_get_timer_counter = 0x0001dab9;
platform_get_us_time = 0x0001dabd;
platform_get_version = 0x0001dac1;
platform_hrng = 0x0001dac9;
platform_install_isr_stack = 0x0001dad1;
platform_install_task_stack = 0x0001dadd;
platform_patch_rf_init_data = 0x0001db15;
platform_printf = 0x0001db21;
platform_raise_assertion = 0x0001db35;
platform_rand = 0x0001db49;
platform_read_info = 0x0001db4d;
platform_read_persistent_reg = 0x0001db7d;
platform_reset = 0x0001db8d;
platform_rt_rc_auto_tune = 0x0001dbb1;
platform_rt_rc_auto_tune2 = 0x0001dbb9;
platform_rt_rc_tune = 0x0001dc35;
platform_set_abs_timer = 0x0001dc59;
platform_set_evt_callback = 0x0001dc5d;
platform_set_evt_callback_table = 0x0001dc71;
platform_set_irq_callback = 0x0001dc7d;
platform_set_irq_callback_table = 0x0001dc99;
platform_set_rf_clk_source = 0x0001dca5;
platform_set_rf_init_data = 0x0001dcb1;
platform_set_rf_power_mapping = 0x0001dcbd;
platform_set_timer = 0x0001dcc9;
platform_shutdown = 0x0001dccd;
platform_switch_app = 0x0001dcd1;
platform_trace_raw = 0x0001dcfd;
platform_write_persistent_reg = 0x0001dd15;
printf_hexdump = 0x0001dec9;
pvPortMalloc = 0x0001e9bd;
pvTaskIncrementMutexHeldCount = 0x0001eaa5;
pvTimerGetTimerID = 0x0001eabd;
pxPortInitialiseStack = 0x0001eae9;
reverse_128 = 0x0001ec91;
reverse_24 = 0x0001ec97;
reverse_256 = 0x0001ec9d;
reverse_48 = 0x0001eca3;
reverse_56 = 0x0001eca9;
reverse_64 = 0x0001ecaf;
reverse_bd_addr = 0x0001ecb5;
reverse_bytes = 0x0001ecbb;
sm_add_event_handler = 0x0001ee59;
sm_address_resolution_lookup = 0x0001efb1;
sm_authenticated = 0x0001f359;
sm_authorization_decline = 0x0001f367;
sm_authorization_grant = 0x0001f387;
sm_authorization_state = 0x0001f3a7;
sm_bonding_decline = 0x0001f3c1;
sm_config = 0x0001f81d;
sm_config_conn = 0x0001f851;
sm_encryption_key_size = 0x0001fa0b;
sm_just_works_confirm = 0x0001ffa1;
sm_le_device_key = 0x000202dd;
sm_passkey_input = 0x00020373;
sm_private_random_address_generation_get = 0x00020749;
sm_private_random_address_generation_get_mode = 0x00020751;
sm_private_random_address_generation_set_mode = 0x0002075d;
sm_private_random_address_generation_set_update_period = 0x00020785;
sm_register_external_ltk_callback = 0x000208c1;
sm_register_oob_data_callback = 0x000208cd;
sm_request_pairing = 0x000208d9;
sm_send_security_request = 0x000213c7;
sm_set_accepted_stk_generation_methods = 0x000213ed;
sm_set_authentication_requirements = 0x000213f9;
sm_set_encryption_key_size_range = 0x00021409;
sscanf_bd_addr = 0x00021755;
sysSetPublicDeviceAddr = 0x00021b51;
uuid128_to_str = 0x00022179;
uuid_add_bluetooth_prefix = 0x000221d1;
uuid_has_bluetooth_prefix = 0x000221f1;
uxListRemove = 0x0002220d;
uxQueueMessagesWaiting = 0x00022235;
uxQueueMessagesWaitingFromISR = 0x0002225d;
uxQueueSpacesAvailable = 0x00022279;
uxTaskGetStackHighWaterMark = 0x000222a5;
uxTaskPriorityGet = 0x000222c5;
uxTaskPriorityGetFromISR = 0x000222e1;
vListInitialise = 0x0002239b;
vListInitialiseItem = 0x000223b1;
vListInsert = 0x000223b7;
vListInsertEnd = 0x000223e7;
vPortEndScheduler = 0x00022401;
vPortEnterCritical = 0x0002242d;
vPortExitCritical = 0x00022471;
vPortFree = 0x000224a5;
vPortSuppressTicksAndSleep = 0x00022539;
vPortValidateInterruptPriority = 0x00022661;
vQueueDelete = 0x000226bd;
vQueueWaitForMessageRestricted = 0x000226e9;
vTaskDelay = 0x00022731;
vTaskInternalSetTimeOutState = 0x0002277d;
vTaskMissedYield = 0x0002278d;
vTaskPlaceOnEventList = 0x00022799;
vTaskPlaceOnEventListRestricted = 0x000227d1;
vTaskPriorityDisinheritAfterTimeout = 0x00022811;
vTaskPrioritySet = 0x000228bd;
vTaskResume = 0x00022985;
vTaskStartScheduler = 0x00022a09;
vTaskStepTick = 0x00022a99;
vTaskSuspend = 0x00022ac9;
vTaskSuspendAll = 0x00022b85;
vTaskSwitchContext = 0x00022b95;
xPortStartScheduler = 0x00022c3d;
xQueueAddToSet = 0x00022d05;
xQueueCreateCountingSemaphore = 0x00022d29;
xQueueCreateCountingSemaphoreStatic = 0x00022d65;
xQueueCreateMutex = 0x00022da9;
xQueueCreateMutexStatic = 0x00022dbf;
xQueueCreateSet = 0x00022dd9;
xQueueGenericCreate = 0x00022de1;
xQueueGenericCreateStatic = 0x00022e2d;
xQueueGenericReset = 0x00022e95;
xQueueGenericSend = 0x00022f21;
xQueueGenericSendFromISR = 0x0002308d;
xQueueGiveFromISR = 0x0002314d;
xQueueGiveMutexRecursive = 0x000231f1;
xQueueIsQueueEmptyFromISR = 0x00023231;
xQueueIsQueueFullFromISR = 0x00023255;
xQueuePeek = 0x0002327d;
xQueuePeekFromISR = 0x000233a5;
xQueueReceive = 0x00023411;
xQueueReceiveFromISR = 0x0002353d;
xQueueRemoveFromSet = 0x000235d1;
xQueueSelectFromSet = 0x000235f3;
xQueueSelectFromSetFromISR = 0x00023605;
xQueueSemaphoreTake = 0x00023619;
xQueueTakeMutexRecursive = 0x00023785;
xTaskCheckForTimeOut = 0x000237c9;
xTaskCreate = 0x00023839;
xTaskCreateStatic = 0x00023895;
xTaskGetCurrentTaskHandle = 0x00023905;
xTaskGetSchedulerState = 0x00023911;
xTaskGetTickCount = 0x0002392d;
xTaskGetTickCountFromISR = 0x00023939;
xTaskIncrementTick = 0x00023949;
xTaskPriorityDisinherit = 0x00023a15;
xTaskPriorityInherit = 0x00023aa9;
xTaskRemoveFromEventList = 0x00023b3d;
xTaskResumeAll = 0x00023bbd;
xTaskResumeFromISR = 0x00023c85;
xTimerCreate = 0x00023d11;
xTimerCreateStatic = 0x00023d45;
xTimerCreateTimerTask = 0x00023d7d;
xTimerGenericCommand = 0x00023de9;
xTimerGetExpiryTime = 0x00023e59;
xTimerGetTimerDaemonTaskHandle = 0x00023e79;