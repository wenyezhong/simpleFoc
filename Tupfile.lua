
-- Utility functions -----------------------------------------------------------

function run_now(command)
    local handle
    handle = io.popen(command)
    local output = handle:read("*a")
    local rc = {handle:close()}
    return rc[1], output
end

-- If we simply invoke python or python3 on a pristine Windows 10, it will try
-- to open the Microsoft Store which will not work and hang tup instead. The
-- command "python --version" does not open the Microsoft Store.
-- On some systems this may return a python2 command if Python3 is not installed.
function find_python3()
    success, python_version = run_now("python --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python -B" end
    success, python_version = run_now("python3 --version 2>&1")
    if success and string.match(python_version, "Python 3") then return "python3 -B" end
    error("Python 3 not found.")
end

function add_pkg(pkg)
    if pkg.is_included == true then
        return
    end
    pkg.is_included = true
    for _, file in pairs(pkg.code_files or {}) do
        code_files += (pkg.root or '.')..'/'..file
    end
    for _, dir in pairs(pkg.include_dirs or {}) do
        CFLAGS += '-I'..(pkg.root or '.')..'/'..dir
    end
    tup.append_table(CFLAGS, pkg.cflags or {})
    tup.append_table(LDFLAGS, pkg.ldflags or {})
    for _, pkg in pairs(pkg.include or {}) do
        add_pkg(pkg)
    end
end

function compile(src_file, obj_file)
    compiler = (tup.ext(src_file) == 'c') and CC or CXX
    tup.frule{
        inputs={src_file},        
        command='^o^ '..compiler..' -c %f '..tostring(CFLAGS)..' -o %o',
        outputs={obj_file}
    }
end

-- Packages --------------------------------------------------------------------




stm32f4xx_hal_pkg = {
    root = 'Board/Drivers/STM32F4xx_HAL_Driver',
    include_dirs = {
        'Inc',
        -- '../CMSIS/Device/ST/STM32F4xx/Include',
        -- '../CMSIS/Include',
    },
    code_files = {
        'Src/stm32f4xx_hal.c',        
        'Src/stm32f4xx_hal_cortex.c',     
        'Src/stm32f4xx_hal_dma.c',
        'Src/stm32f4xx_hal_dma_ex.c',
        'Src/stm32f4xx_hal_adc_ex.c',
        'Src/stm32f4xx_hal_adc.c',
        'Src/stm32f4xx_hal_flash.c',
        'Src/stm32f4xx_hal_flash_ex.c',
        'Src/stm32f4xx_hal_flash_ramfunc.c',
        'Src/stm32f4xx_hal_gpio.c',
        'Src/stm32f4xx_hal_pwr.c',
        'Src/stm32f4xx_hal_pwr_ex.c',
        'Src/stm32f4xx_hal_rcc.c',
        'Src/stm32f4xx_hal_rcc_ex.c',
        'Src/stm32f4xx_hal_uart.c',
		'Src/stm32f4xx_hal_tim.c',
        'Src/stm32f4xx_hal_tim_ex.c',
		'Src/stm32f4xx_hal_exti.c',
        'Src/stm32f4xx_hal_spi.c',
        'Src/stm32f4xx_ll_usb.c', 
        'Src/stm32f4xx_hal_pcd_ex.c',
        'Src/stm32f4xx_hal_pcd.c',
    },
    cflags = {'-DARM_MATH_CM4', '-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-DFPU_FPV4'}
}

cmsis_pkg = {
    root = 'Board/Drivers/CMSIS',
    include_dirs = {
        'Include',        
        'Device/ST/STM32F4xx/Include'        
    }, 
    ldflags = {'-LBoard/Middlewares/ST/ARM/DSP/Lib'},   
}

stm32_usb_device_library_pkg = {
    root = 'Board/Middlewares/ST/STM32_USB_Device_Library',
    include_dirs = {
        'Core/Inc',
        'Class/CDC/Inc',
    },
    code_files = {
        'Core/Src/usbd_core.c',
        'Core/Src/usbd_ctlreq.c',
        'Core/Src/usbd_ioreq.c',
        'Class/CDC/Src/usbd_cdc.c',
    }
}
simpleFoc_firmware_pkg = {
    root = '.',
    include_dirs = {
        '.',       
        'Board/Middlewares/ST/ARM/DSP/Inc',
        'arduino',
        'communication',
        'common',
        'common/base_classes',
        'MotorControl',
        'current_sense',
        'drivers',
        'drivers/hardware_specific',
    },
    code_files = {
        'arduino/Print.cpp',
        'common/base_classes/CurrentSense.cpp',
        'common/base_classes/FOCMotor.cpp',
        'common/base_classes/Sensor.cpp',
        'communication/SimpleFOCDebug.cpp',
        'communication/Commander.cpp',
        'common/foc_utils.cpp',
        'common/lowpass_filter.cpp',
        'common/pid.cpp',
        'common/time_utils.cpp',        
        'sensors/Encoder.cpp',
        'drivers/hardware_specific/stm32_mcu.cpp',
        'MotorControl/main.cpp',
        'MotorControl/BLDCMotor.cpp',
        'drivers/BLDCDriver3PWM.cpp',
        'current_sense/LowsideCurrentSense.cpp',
        'current_sense/stm32f4_mcu.cpp',
    },
    cflags = {'-D_STM32_DEF_'},  
}

board_v3 = {
    root = 'Board/Core',
    include = {stm32f4xx_hal_pkg},
    include_dirs = {
        'Inc',
        '../../drivers/DRV8301',
        'common/base_classes',
        '../USB_DEVICE/App',
        '../USB_DEVICE/Target',
        -- 'Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F', 
    },
    code_files = {
        '../startup_stm32f405xx.s',
        -- './Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c',
        '../../drivers/DRV8301/drv8301.c', 
        'Src/adc.c',        
        'Src/tim.c',
        'Src/dma.c',
        'Src/gpio.c',
        'Src/main.c',
        --[[ 'Src/stm32f4xx_hal_msp.c',
        'Src/stm32f4xx_hal_timebase_TIM.c', ]]
        'Src/stm32f4xx_it.c',
        'Src/system_stm32f4xx.c',
        'Src/usart.c',
        'Src/spi.c',
        '../USB_DEVICE/App/usb_device.c',
        '../USB_DEVICE/App/usbd_cdc_if.c',
        '../USB_DEVICE/App/usbd_desc.c',
        '../USB_DEVICE/Target/usbd_conf.c',
        --[[ 'Src/usb_device.c',
        'Src/usbd_cdc_if.c',
        'Src/usbd_conf.c',              
        'Src/usbd_desc.c',
        'Src/can.c',   ]]
        --'Src/i2c.c',      
    },
    cflags = {'-DSTM32F405xx', '-DHW_VERSION_MAJOR=3'},    
    ldflags = {
        '-TBoard/STM32F405RGTx_FLASH.ld',
        '-l:iar_cortexM4lf_math.a',
    }
}


boards = {
    ["v3.1"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=1 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.2"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=2 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.3"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=3 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.4-24V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=4 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.4-48V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=4 -DHW_VERSION_VOLTAGE=48"}},
    ["v3.5-24V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=5 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.5-48V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=5 -DHW_VERSION_VOLTAGE=48"}},
    ["v3.6-24V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=6 -DHW_VERSION_VOLTAGE=24"}},
    ["v3.6-56V"] = {include={board_v3}, cflags={"-DHW_VERSION_MINOR=6 -DHW_VERSION_VOLTAGE=56"}},
}


-- Toolchain setup -------------------------------------------------------------

CC='arm-none-eabi-gcc -std=c99'
CXX='arm-none-eabi-g++ -std=c++17 -Wno-register'
LINKER='arm-none-eabi-g++'

-- C-specific flags
CFLAGS += '-D__weak="__attribute__((weak))"'
CFLAGS += '-D__packed="__attribute__((__packed__))"'
CFLAGS += '-DUSE_HAL_DRIVER'

CFLAGS += '-mthumb'
CFLAGS += '-mfloat-abi=hard'
CFLAGS += '-Wno-psabi' -- suppress unimportant note about ABI compatibility in GCC 10
-- CFLAGS += { '-Wall', '-Wdouble-promotion', '-Wfloat-conversion', '-fdata-sections', '-ffunction-sections'}
CFLAGS += { '-w', '-Wdouble-promotion', '-Wfloat-conversion', '-fdata-sections', '-ffunction-sections'}
CFLAGS += '-g'
CFLAGS += '-DFIBRE_ENABLE_SERVER'
CFLAGS += '-Wno-nonnull'

-- linker flags
LDFLAGS += '-flto -lc -lm -lnosys' -- libs
-- LDFLAGS += '-mthumb -mfloat-abi=hard -specs=nosys.specs -specs=nano.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-mthumb -mfloat-abi=hard -specs=nosys.specs -u _printf_float -u _scanf_float -Wl,--cref -Wl,--gc-sections'
LDFLAGS += '-Wl,--undefined=uxTopUsedPriority'


-- Handle Configuration Options ------------------------------------------------

-- Switch between board versions
boardversion = tup.getconfig("BOARD_VERSION")
if boardversion == "" then
    error("board version not specified - take a look at tup.config.default")
elseif boards[boardversion] == nil then
    error("unknown board version "..boardversion)
end
board = boards[boardversion]

-- --not 
-- TODO: remove this setting
if tup.getconfig("USB_PROTOCOL") ~= "native" and tup.getconfig("USB_PROTOCOL") ~= "" then
    error("CONFIG_USB_PROTOCOL is deprecated")
end

-- UART I/O settings
if tup.getconfig("UART_PROTOCOL") ~= "ascii" and tup.getconfig("UART_PROTOCOL") ~= "" then
    error("CONFIG_UART_PROTOCOL is deprecated")
end

-- Compiler settings
if tup.getconfig("STRICT") == "true" then
    CFLAGS += '-Werror'
end

if tup.getconfig("NO_DRM") == "true" then
    CFLAGS += '-DNO_DRM'
end

-- debug build
if tup.getconfig("DEBUG") == "true" then
    CFLAGS += '-gdwarf-2 -Og'
else
    CFLAGS += '-O2'
end

if tup.getconfig("USE_LTO") == "true" then
    CFLAGS += '-flto'
end


-- Generate Tup Rules ----------------------------------------------------------

python_command = find_python3()
print('Using python command "'..python_command..'"')

-- TODO: use CI to verify that on PRs the enums.py file is consistent with the YAML.
-- Note: we currently check this file into source control for two reasons:
--  - Don't require tup to run in order to use odrivetool from the repo
--  - On Windows, tup is unhappy with writing outside of the tup directory
--tup.frule{command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template enums_template.j2 --output ../tools/odrive/enums.py'}

-- tup.frule{
--     command=python_command..' ../tools/odrive/version.py --output %o',
--     outputs={'autogen/version.c'}
-- }

-- Autogen files from YAML interface definitions
--root_interface = board.include[1].root_interface
--tup.frule{inputs={'fibre-cpp/interfaces_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/interfaces.hpp'}
--tup.frule{inputs={'fibre-cpp/function_stubs_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/function_stubs.hpp'}
--tup.frule{inputs={'fibre-cpp/endpoints_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --generate-endpoints '..root_interface..' --template %f --output %o', outputs='autogen/endpoints.hpp'}
--tup.frule{inputs={'fibre-cpp/type_info_template.j2', extra_inputs='odrive-interface.yaml'}, command=python_command..' interface_generator_stub.py --definitions odrive-interface.yaml --template %f --output %o', outputs='autogen/type_info.hpp'}

add_pkg(board)
-- add_pkg(freertos_pkg)
add_pkg(cmsis_pkg)
add_pkg(stm32_usb_device_library_pkg)
--add_pkg(fibre_pkg)
add_pkg(simpleFoc_firmware_pkg)


for _, src_file in pairs(code_files) do
    obj_file = "build/obj/"..src_file:gsub("/","_"):gsub("%.","")..".o"
    object_files += obj_file
    compile(src_file, obj_file)
end

tup.frule{
    inputs=object_files,
    command='^o^ '..LINKER..' %f '..tostring(CFLAGS)..' '..tostring(LDFLAGS)..
            ' -Wl,-Map=%O.map -o %o',
    outputs={'build/simpleFoc.elf', extra_outputs={'build/simpleFoc.map'}}
}
-- display the size
tup.frule{inputs={'build/simpleFoc.elf'}, command='arm-none-eabi-size %f'}
-- create *.hex and *.bin output formats
tup.frule{inputs={'build/simpleFoc.elf'}, command='arm-none-eabi-objcopy -O ihex %f %o', outputs={'build/simpleFoc.hex'}}
tup.frule{inputs={'build/simpleFoc.elf'}, command='arm-none-eabi-objcopy -O binary -S %f %o', outputs={'build/simpleFoc.bin'}}

if tup.getconfig('ENABLE_DISASM') == 'true' then
    tup.frule{inputs={'build/simpleFoc.elf'}, command='arm-none-eabi-objdump %f -dSC > %o', outputs={'build/simpleFoc.asm'}}
end

if tup.getconfig('DOCTEST') == 'true' then
    TEST_INCLUDES = '-I. -I./MotorControl -I./fibre-cpp/include -I./Drivers/DRV8301 -I./doctest'
    tup.foreach_rule('Tests/*.cpp', 'g++ -O3 -std=c++17 '..TEST_INCLUDES..' -c %f -o %o', 'Tests/bin/%B.o')
    tup.frule{inputs='Tests/bin/*.o', command='g++ %f -o %o', outputs='Tests/test_runner.exe'}
    tup.frule{inputs='Tests/test_runner.exe', command='%f'}
end
