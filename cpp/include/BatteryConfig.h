#pragma once

struct BatteryConfig {
    // Full-pack values under the expected payload and flight conditions.
    double capacityWh{220.0};
    double usableFlightTimeSeconds{900.0};

    // Operational allowances applied to every takeoff-to-landing sortie.
    double reserveFraction{0.20};
    double secondsPerTurn{1.5};
    double takeoffLandingTimeSeconds{45.0};
};
