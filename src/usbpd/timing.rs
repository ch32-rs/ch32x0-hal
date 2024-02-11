//! Table 6.69 “Time Values”

/*
tNoResponse   4.5 5.0 5.5 s


tReceive     0.9 1.0 1.1
tReceiverResponse         15 ms

tRetry         195


tSenderResponse 27 30 33 ms

tEnterEPR
*/

use embassy_time::Duration;

/// tPPSRequest = 10 s
pub const PPS_REQUEST_TIMEOUT: Duration = Duration::from_secs(10 - 2);
