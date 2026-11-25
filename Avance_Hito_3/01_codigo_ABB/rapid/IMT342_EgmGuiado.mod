MODULE IMT342_EgmGuiado
    ! Modulo candidato para validar en un controlador virtual RW 6.12.
    ! Llamar desde T_ROB1 despues de un punto fine ya validado.
    ! El cliente debe enviar referencias articulares verificadas, no XYZ crudo.
    LOCAL CONST egm_minmax imtConvergence := [-0.1,0.1];

    PROC IMT342_GuiarJuntas(string udpDevice, string motionProfile,
                            num speedPct, num cellSpeedCap, num maxCorrection,
                            num commTimeout)
        VAR egmident guidanceId;
        VAR bool reserved := FALSE;
        VAR num jointSpeed;

        IF udpDevice = "" OR motionProfile = "" THEN
            TPWrite "Falta dispositivo UDP o perfil EGM.";
            RETURN;
        ENDIF
        IF speedPct <= 0 OR speedPct > 100 OR cellSpeedCap <= 0 OR
           maxCorrection <= 0 OR commTimeout <= 0 THEN
            TPWrite "Parametros EGM invalidos.";
            RETURN;
        ENDIF

        ! EGM aplica un solo limite comun en grados/s. Se toma el
        ! menor maximo nominal ABB (250 grados/s) y el limite de la celda.
        jointSpeed := 250 * speedPct / 100;
        IF jointSpeed > cellSpeedCap THEN
            jointSpeed := cellSpeedCap;
        ENDIF
        IF jointSpeed > 50 THEN
            jointSpeed := 50;
        ENDIF

        EGMGetId guidanceId;
        reserved := TRUE;
        EGMSetupUC ROB_1, guidanceId, motionProfile, udpDevice\Joint
                  \CommTimeout:=commTimeout;
        EGMActJoint guidanceId\Tool:=tool0
                   \J1:=imtConvergence\J2:=imtConvergence\J3:=imtConvergence
                   \J4:=imtConvergence\J5:=imtConvergence\J6:=imtConvergence
                   \MaxPosDeviation:=maxCorrection\MaxSpeedDeviation:=jointSpeed;
        EGMRunJoint guidanceId, EGM_STOP_HOLD\J1\J2\J3\J4\J5\J6
                   \CondTime:=0.5;
        EGMReset guidanceId;
        reserved := FALSE;
    ERROR
        IF reserved THEN
            EGMReset guidanceId;
        ENDIF
        RAISE;
    ENDPROC
ENDMODULE
