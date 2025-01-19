package frc.robot.utility;

import edu.wpi.first.wpilibj.DriverStation;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Collections;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

/**
 * Utilities for retrieving the MAC addresses of the current system.
 */
public final class MacAddressUtils {
    private MacAddressUtils() {
        throw new UnsupportedOperationException("MacAddressUtils is a utility class and cannot be instantiated");
    }

    /**
     * Retrieves the MAC addresses of all the network interfaces of the current system.
     *
     * @return The MAC addresses of the system's network adapters.
     */
    public static Set<String> getMacAddresses() {
        try {
            return NetworkInterface.networkInterfaces()
                    .flatMap(iface -> {
                        try {
                            byte[] address = iface.getHardwareAddress();

                            // If the interface doesn't have a hardware address, skip it.
                            if (address == null) {
                                return Stream.empty();
                            }

                            // Build the MAC address string
                            StringBuilder builder = new StringBuilder();
                            for (int i = 0; i < address.length; i++) {
                                if (i != 0) {
                                    builder.append('-');
                                }
                                // Turn each byte of the MAC address into a 2-digit hex number
                                builder.append(String.format("%02x", address[i]));
                            }

                            return Stream.of(builder.toString());
                        } catch (SocketException e) {
                            return Stream.empty();
                        }
                    })
                    .collect(Collectors.toSet());
        } catch (SocketException e) {
            DriverStation.reportError("Failed to retrieve MAC address", e.getStackTrace());

            return Collections.emptySet();
        }
    }
}
