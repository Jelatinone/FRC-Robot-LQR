// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertThrows;
import com.revrobotics.REVLibError;
import com.ctre.phoenix.ErrorCode;
import org.junit.jupiter.api.Test;
// ------------------------------------------------------------[Motor Test Class]-----------------------------------------------------------//
public final class UtilitiesTest {

  @Test
  void CheckErrorCTRE() {
    assertThrows(RuntimeException.class, () -> CheckCTRECode(ErrorCode.GeneralError));
    assertThrows(RuntimeException.class, () -> CheckCTRECode(ErrorCode.FirmVersionCouldNotBeRetrieved));
    assertDoesNotThrow(() -> CheckCTRECode(ErrorCode.OK));
  }

  static void CheckCTRECode(final ErrorCode errorCode) {
      if (errorCode != ErrorCode.OK) {
        throw new RuntimeException(String.format("%s: %s%n", "", errorCode.toString()));
        
    }
  }

  @Test
  void CheckErrorREV() {
    assertThrows(RuntimeException.class, () -> CheckREVLibCode(REVLibError.kError));
    assertThrows(RuntimeException.class, () -> CheckREVLibCode(REVLibError.kCantFindFirmware));
    assertDoesNotThrow(() -> CheckREVLibCode(REVLibError.kOk));
  }

  static void CheckREVLibCode(final REVLibError error) {
    if (error != REVLibError.kOk) {
        throw new RuntimeException(String.format("%s: %s", "", error.toString()));
    }
  }
}
