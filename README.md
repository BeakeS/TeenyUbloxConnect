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
    
    // Get the latest Position/Velocity/Time solution and fill all global variables
    // Returns true when a packet has been received
    bool    getNAVPVT(); // Use only when autoNAVPVTRate > 0
    bool    pollNAVPVT(uint16_t maxWait_ = defaultMaxWait); // Use only when autoPVTRate = 0
    
    // Get the latest satellite information
    //Returns true when a packet has been received
    bool    getNAVSAT(); // Use only when autoNAVSATRate > 0
    bool    pollNAVSAT(uint16_t maxWait_ = defaultMaxWait); // Use only when autoNAVSATRate = 0

    // Ublox command methods
    void    setSerialRate(uint32_t baudrate_, uint8_t uartPort_ = COM_PORT_UART1, uint16_t maxWait_ = defaultMaxWait);
    bool    saveConfiguration(uint16_t maxWait_ = defaultMaxWait);
    bool    getProtocolVersion(uint16_t maxWait_ = defaultMaxWait);
    uint8_t getProtocolVersionHigh(uint16_t maxWait_ = defaultMaxWait);
    uint8_t getProtocolVersionLow(uint16_t maxWait_ = defaultMaxWait);
    bool    setPortOutput(uint8_t portID_, uint8_t comSettings_, uint16_t maxWait_ = defaultMaxWait);
    bool    setMeasurementRate(uint16_t rate_, uint16_t maxWait_ = defaultMaxWait);
    bool    setNavigationRate(uint16_t rate_, uint16_t maxWait_ = defaultMaxWait);
    bool    setAutoNAVPVT(bool enable_, uint16_t maxWait_ = defaultMaxWait); // same as setAutoNAVPVTRate = 0(false) or 1(true)
    bool    setAutoNAVPVTRate(uint8_t rate_, uint16_t maxWait_ = defaultMaxWait);
    bool    setAutoNAVSAT(bool enable_, uint16_t maxWait_ = defaultMaxWait); // same as setAutoNAVSATRate = 0(false) or 1(true)
    bool    setAutoNAVSATRate(uint8_t rate_, uint16_t maxWait_ = defaultMaxWait);

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
    void     getNAVSATInfo(ubloxNAVSATInfo_t &info_); // summary and sorted sat details

    // Access lost packet counts
    uint8_t  getLostRxPacketCount();
    uint8_t  getLostNAVPVTPacketCount();
    uint8_t  getLostNAVSATPacketCount();

