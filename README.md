# TeenyUbloxConnect

The TeenyUbloxConnect library is derived from the SparkFun_u-blox_GNSS_Arduino_Library.
This library only supports serial communication with u-blox GNSS modules.
This library only processes UBX-NAV-PVT and UBX-NAV-SAT packets and provides raw packet data and derived packet information.
An example of a project that uses this library can be found here: https://github.com/BeakeS/TeenyGPSTestbed_C2.git

    // Ublox setup
    bool    begin(Stream &serialPort_, uint16_t maxWait_ = defaultMaxWait); // default maxWait is 1100ms

    // Host methods for process incoming responses/acknowledges from ublox receiver
    // Can be called inside a timer ISR
    // Recommend calling ever 10-50ms - depends on queue size, baud rate and packets
    void    checkUblox();

    // Ublox command methods
    bool    pollUART1Port(uint16_t maxWait_ = defaultMaxWait);
    bool    setPortOutput(uint8_t portID_, uint8_t comSettings_, uint16_t maxWait_ = defaultMaxWait);
    void    setSerialRate(uint32_t baudrate_, uint8_t uartPort_ = COM_PORT_UART1, uint16_t maxWait_ = defaultMaxWait);
    void    hardwareReset();
    void    coldStart();
    void    warmStart();
    void    hotStart();
    bool    saveConfiguration(uint32_t configMask = 0xFFFF, uint16_t maxWait_ = defaultMaxWait);
    bool    pollProtocolVersion(uint16_t maxWait_ = defaultMaxWait);
    uint8_t getProtocolVersionHigh(uint16_t maxWait_ = defaultMaxWait);
    uint8_t getProtocolVersionLow(uint16_t maxWait_ = defaultMaxWait);
    bool    pollGNSSSelectionInfo(uint16_t maxWait_ = defaultMaxWait);
    bool    pollGNSSConfigInfo(uint16_t maxWait_ = defaultMaxWait);
    bool    setGNSSConfig(uint8_t gnssId, bool enable, uint16_t maxWait_ = defaultMaxWait);
    bool    setMeasurementRate(uint16_t rate_, uint16_t maxWait_ = defaultMaxWait);
    bool    setNavigationRate(uint16_t rate_, uint16_t maxWait_ = defaultMaxWait);
    bool    setAutoNAVPVT(bool enable_, uint16_t maxWait_ = defaultMaxWait);
    bool    setAutoNAVPVTRate(uint8_t rate_, uint16_t maxWait_ = defaultMaxWait);
    bool    setAutoPVT(bool enable_, uint16_t maxWait_ = defaultMaxWait); // Same as setAutoNAVPVT
    bool    setAutoPVTRate(uint8_t rate_, uint16_t maxWait_ = defaultMaxWait); // Same as setAutoNAVPVTRate
    bool    setAutoNAVSAT(bool enable_, uint16_t maxWait_ = defaultMaxWait);
    bool    setAutoNAVSATRate(uint8_t rate_, uint16_t maxWait_ = defaultMaxWait);
    
    // Get the latest Position/Velocity/Time solution and fill all global variables
    // Returns true when a packet has been received
    bool    getNAVPVT(); // Use only when autoNAVPVTRate > 0
    bool    pollNAVPVT(uint16_t maxWait_ = defaultMaxWait); // Use only when autoPVTRate = 0
    
    // Get the latest satellite information
    //Returns true when a packet has been received
    bool    getNAVSAT(); // Use only when autoNAVSATRate > 0
    bool    pollNAVSAT(uint16_t maxWait_ = defaultMaxWait); // Use only when autoNAVSATRate = 0

    // Ublox GNSS info data access
    ubloxMONGNSSInfo_t getGNSSSelectionInfo();
    ubloxCFGGNSSInfo_t getGNSSConfigInfo();

    // Ublox navpvt data access
    void     getNAVPVTPacket(uint8_t *packet_); // Get the full NAV-PVT packet
    uint16_t getYear();
    uint8_t  getMonth();
    uint8_t  getDay();
    uint8_t  getHour();
    uint8_t  getMinute();
    uint8_t  getSecond();
    bool     getDateValid();
    bool     getTimeValid();
    uint32_t getTimeAccEst();
    uint8_t  getFixType();
    bool     getGnssFixOk();
    uint8_t  getSIV();
    int32_t  getLongitude();
    int32_t  getLatitude();
    int32_t  getAltitudeMSL();
    uint32_t getHorizontalAccEst();
    uint32_t getVerticalAccEst();
    int32_t  getHeading();
    uint16_t getPDOP();

    // Ublox navsat data access
    void     getNAVSATPacket(ubloxPacket_t &packet_); // Get the full NAV-SAT packet
    uint16_t getNAVSATPacketLength(); // Get the actual NAV-SAT packet length
    void     getNAVSATInfo(ubloxNAVSATInfo_t &info_); // summary and sorted sat details

    // Access lost packet counts
    uint8_t  getLostRxPacketCount();
    uint8_t  getUnknownRxPacketCount();
    uint8_t  getLostNAVPVTPacketCount();
    uint8_t  getLostNAVSATPacketCount();

