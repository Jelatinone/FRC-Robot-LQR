// ----------------------------------------------------------------[Package]----------------------------------------------------------------//
package frc.robot.tests;
// ---------------------------------------------------------------[Libraries]---------------------------------------------------------------//
//import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
//import static org.junit.jupiter.api.Assertions.assertThrows;
//import com.revrobotics.REVLibError;
//import com.ctre.phoenix.ErrorCode;
//import org.junit.jupiter.api.Test;
// ----------------------------------------------------------[Robot Container Class]--------------------------------------------------------//
public final class UtilitiesTest {

  //@Test
  static void CheckErrorCTRE() {
    //assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(ErrorCode.GeneralError, ""));
    //assertThrows(RuntimeException.class, () -> CtreUtils.checkCtreError(ErrorCode.FirmVersionCouldNotBeRetrieved, ""));
    //assertDoesNotThrow(() -> CtreUtils.checkCtreError(ErrorCode.OK, ""));
  }

  //@Test
  static void CheckErrorREV() {
    //assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(REVLibError.kError, ""));
    //assertThrows(RuntimeException.class, () -> RevUtils.checkNeoError(REVLibError.kCantFindFirmware, ""));
    //assertDoesNotThrow(() -> RevUtils.checkNeoError(REVLibError.kOk, ""));
  }
}
