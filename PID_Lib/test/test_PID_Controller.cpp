/// \file test_TE_PID_Controller.cpp
/// The arrays used to test the PI and PID controllers are generated in the PID_ControllerTest.m and ValidatePiWIthAntiWindup.m scripts.
/// Author: Charl van de Merwe
/// Date: 12 October 2021

#include <catch2/catch.hpp>
// Include the header files of the units to be tested here
#include "PID_Controller.h"

namespace {
    constexpr TE::float32_t PI {3.141592653589793238462643383279502884L};

    constexpr TE::float32_t REF {1.0F}; ///< Angular rate reference

    /// Plant output (used to calculate the error for the input to the PI controller)
    constexpr TE::float32_t yPI[] = {0.0F,
                                     0.377283476821304F,
                                     0.645631389287896F,
                                     0.833539319597913F,
                                     0.962391176076050F,
                                     1.04819035096566F,
                                     1.10287532251065F,
                                     1.13531865039622F,
                                     1.15208486434512F,
                                     1.15800483780016F,
                                     1.15661056485816F,
                                     1.15046381975235F,
                                     1.14140421053942F,
                                     1.13073605861393F,
                                     1.11936889726116F,
                                     1.10792284488688F,
                                     1.09680741145666F,
                                     1.08628024112511F,
                                     1.07649072809222F,
                                     1.06751225033795F,
                                     1.05936585843552F,
                                     1.05203756646028F,
                                     1.04549086743951F,
                                     1.03967569740315F,
                                     1.03453476981270F,
                                     1.03000797301693F,
                                     1.02603534990634F,
                                     1.02255904777256F,
                                     1.01952452735514F,
                                     1.01688124543278F,
                                     1.01458296919085F};

    /// Expected control output (input to the plant) of the PI controller
    constexpr TE::float32_t uPI[] = {0.111873000000000F, 0.0795711655975702F, 0.0557189094648033F, 0.0382074610349450F,
                                     0.0254413767952295F, 0.0162153187125950F, 0.00962017327427575F, 0.00497155790894298F,
                                     0.00175540470501345F, -0.000413433151525332F, -0.00182264757793317F, -0.00268637701819567F,
                                     -0.00316334595518486F, -0.00337061790442780F, -0.00339401085903891F, -0.00329597493804069F,
                                     -0.00312154175535051F, -0.00290280984673888F, -0.00266231972379570F, -0.00241558763447390F,
                                     -0.00217300268502104F, -0.00194124287053681F, -0.00172432814434189F, -0.00152440015971981F,
                                     -0.00134229662586965F, -0.00117797171770100F, -0.00103080143315569F, -0.000899803260714914F,
                                     -0.000783792285292547F, -0.000681490374772079F, -0.000591600934015324F};

    /// Plant output (used to calculate the error for the input to the PID controller)
    constexpr TE::float32_t yPID[] = {0.0F,
                                      0.395489688235515F,
                                      0.627734709539703F,
                                      0.771429002493312F,
                                      0.865340306230226F,
                                      0.929895661726318F,
                                      0.976111996179741F,
                                      1.01013825637355F,
                                      1.03557141733435F,
                                      1.05464241130483F,
                                      1.06882766054323F,
                                      1.07916804092995F,
                                      1.08643829804083F,
                                      1.09123957111128F,
                                      1.09405199823890F,
                                      1.09526627310073F,
                                      1.09520383123262F,
                                      1.09413066970186F,
                                      1.09226742146156F,
                                      1.08979708634129F,
                                      1.08687119240641F,
                                      1.08361483417558F,
                                      1.08013086070324F,
                                      1.07650339203710F,
                                      1.07280078901013F,
                                      1.06907816931884F,
                                      1.06537954243035F,
                                      1.06173962191666F,
                                      1.05818536365206F,
                                      1.05473727049811F,
                                      1.05141049785474F};

    /// Expected control output (input to the plant) of the PID controller
    constexpr TE::float32_t uPID[] = {0.117271549405615F, 0.0688658498571607F, 0.0426085758407414F, 0.0278468046665509F,
                                      0.0191421085976561F, 0.0137041781629815F, 0.0100895481528477F, 0.00754150179101692F,
                                      0.00565497680003194F, 0.00420624407254608F, 0.00306615435362228F, 0.00215579404807045F,
                                      0.00142368498810945F, 0.000833947626600252F, 0.000360059689770840F, -1.85154122501948e-05F,
                                      -0.000318216426912974F, -0.000552494829989440F, -0.000732509685382976F, -0.000867593075983105F,
                                      -0.000965583140357399F, -0.00103307615682010F, -0.00107562569530401F, -0.00109790471589659F,
                                      -0.00110384010513111F, -0.00109672570179014F, -0.00107931794695444F, -0.00105391717172515F,
                                      -0.00102243683888796F, -0.000986462590574970F, -0.000947302623313802F};

    constexpr std::size_t PID_SAMPLES {sizeof(uPI) / sizeof(uPI[0])}; ///< Amount of PID samples

    /// Input to the PI controller with anti-windup
    constexpr TE::float32_t ERR_WINDUP[] = {
            628.318530717959F, 522.370625584225F, 416.422720450490F, -317.843715401203F, -211.895810267468F, -112.271047137191F,
            -49.2681037815016F, -14.0255663363125F, 4.40896897734108F, 12.9413087257795F, 15.8599933256272F, 15.7775905051781F,
            14.2460853397568F, 12.1558842812777F, 9.99205454280852F, 7.99703464146361F, 6.27255941216896F, 4.84244112469804F,
            3.69041312551559F, 2.78228999250424F, 2.07841920144585F};

    /// Expected output of the PI controller with anti-windup
    constexpr TE::float32_t U_WINDUP[] = {
            15.7079632679490F, 15.7079632679490F, 15.7079632679490F, -15.7079632679490F, -15.7079632679490F, -15.7079632679490F,
            -10.4952687513154F, -7.25973632728620F, -5.38397782498833F, -4.29982820815817F, -3.70245675262123F, -3.39786535675568F,
            -3.26419935612261F, -3.22614574222923F, -3.23810493587495F, -3.27321254626735F, -3.31623883908825F, -3.35904189106644F,
            -3.39769027053641F, -3.43066819883088F, -3.45777587853557F};

    constexpr std::size_t WINDUP_SAMPLES {sizeof(U_WINDUP) / sizeof(U_WINDUP[0])}; ///< Amount of windup samples

} // namespace

namespace CV {
    SCENARIO("Testing the PI(D) controller class") {
        GIVEN("The PI gain values and the control period") {
            constexpr TE::float32_t Kp {0.10692F};
            constexpr TE::float32_t Ki {0.19812F};
            constexpr PI_Params piParams {Kp, Ki, 0.0F};
            constexpr TE::float32_t Ts {1.0F / 20.0F};
            PID_Controller piCtrl {piParams, Ts};
            WHEN("The PI controller object is used in a feedback loop with a step reference") {
                THEN("The PI loop will output the required control values") {
                    for (std::size_t i = 0; i < PID_SAMPLES; i++) {
                        const TE::float32_t PI_Output {piCtrl.runIterFromSetpoint(REF, yPI[i])};
                        REQUIRE(static_cast<std::int32_t>(10000.0F * PI_Output) == static_cast<std::int32_t>(10000.0F * uPI[i]));
                    }
                }
            }
        }
        GIVEN("The PID gain values and the control period") {
            constexpr TE::float32_t Kp {0.0924F};
            constexpr TE::float32_t Ki {0.0999F};
            constexpr TE::float32_t Kd {0.003565F};
            constexpr TE::float32_t N {7.444F};
            constexpr PID_Params pidParams {Kp, Ki, 0.0F, Kd, N};
            constexpr TE::float32_t Ts {1.0F / 20.0F};
            PID_Controller pidCtrl {pidParams, Ts};
            WHEN("The PI controller object is used in a feedback loop with a step reference") {
                THEN("The PI loop will output the required control values") {
                    for (std::size_t i = 0; i < PID_SAMPLES; i++) {
                        const TE::float32_t PID_Output {pidCtrl.runIterFromSetpoint(REF, yPID[i])};
                        REQUIRE(static_cast<std::int32_t>(10000.0F * PID_Output) == static_cast<std::int32_t>(10000.0F * uPID[i]));
                    }
                }
            }
        }
    }

    SCENARIO("Testing the PI(D) controller with internal anti-windup class") {
        GIVEN("The PI gain values, the control period and the controller output limits") {
            constexpr TE::float32_t Kp {0.10692F};
            constexpr TE::float32_t Ki {0.19812F};
            constexpr PI_Params piParams {Kp, Ki, 0.0F};
            constexpr TE::float32_t Ts {0.1F};
            constexpr TE::float32_t bottomLim {-5.0F * PI};
            constexpr TE::float32_t topLim {5.0F * PI};
            constexpr TE::float32_t Ka {1.0F / Kp};
            PID_Ctrl_InternalAntiWindup piCtrl {piParams, Ts, bottomLim, topLim, Ka};
            WHEN("The PI controller with anti-windup object is used in a feedback loop with a step reference") {
                THEN("The PI loop will output the required control values and the integral term will not wind up") {
                    for (std::size_t i = 0; i < WINDUP_SAMPLES; i++) {
                        const TE::float32_t PI_Output {piCtrl.compute(ERR_WINDUP[i])};
                        REQUIRE(static_cast<std::int32_t>(1000.0F * PI_Output) == static_cast<std::int32_t>(1000.0F * U_WINDUP[i]));
                    }
                }
            }
        }
    }
} // namespace CV
