
[SDK Rxxxx]

        1. Source Tree
           User can find source tree as below after the file "loranode_Lxxx_sdk_Rxxxx.tgz" is extracted.
            docs/                                     <-- Documents for SDK
            loranode_ws1510_sdkRxxxx/mbed-os/           <-- mbed os
            loranode_ws1510_sdkRxxxx/libLoraNodeRxxxx.a <-- Lora Node library
            loranode_Lxxx_sdk_Rxxxx/node_api.h          <-- Node API header file
            loranode_Lxxx_sdk_Rxxxx/main.cpp            <-- Sample code


        2. Setup complier environment
           Please follow below link to install compiler.
           https://docs.mbed.com/docs/mbed-os-handbook/en/5.2/dev_tools/cli/


        3. Compile lora node
           $ tar zxvf loranode_Lxxx_sdk_Rxxxx.tgz
           $ cd loranode_Lxxx_sdk_Rxxxx/
           $ ./make.sh

           Then, you will get binary file "loranode_ws1510_sdk_Rxxxx.bin"


	   
	
