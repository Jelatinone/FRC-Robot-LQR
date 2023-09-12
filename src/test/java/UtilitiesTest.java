// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import com.revrobotics.REVLibError;
import com.ctre.phoenix.ErrorCode;
import org.junit.jupiter.api.Test;
// ----------------------------------------------------------[Robot Container Class]--------------------------------------------------------//
public final class UtilitiesTest {

  @Test
  static void CheckErrorCTRE() {
    assertThrows(RuntimeException.class, () -> CheckCTRECode(ErrorCode.GeneralError, ""));
    assertThrows(RuntimeException.class, () -> CheckCTRECode(ErrorCode.FirmVersionCouldNotBeRetrieved, ""));
    assertDoesNotThrow(() -> CheckCTRECode(ErrorCode.OK, ""));
  }

  static void CheckCTRECode(final ErrorCode errorCode, final String message) {
      if (errorCode != ErrorCode.OK) {
        System.out.println(String.format("%s: %s", message, errorCode.toString()));
    }
  }

  @Test
  static void CheckErrorREV() {
    assertThrows(RuntimeException.class, () -> CheckREVLibCode(REVLibError.kError, ""));
    assertThrows(RuntimeException.class, () -> CheckREVLibCode(REVLibError.kCantFindFirmware, ""));
    assertDoesNotThrow(() -> CheckREVLibCode(REVLibError.kOk, ""));
  }

  static void CheckREVLibCode(final REVLibError error, final String message) {
    if (error != REVLibError.kOk) {
        throw new RuntimeException(String.format("%s: %s", message, error.toString()));
    }
}
}
