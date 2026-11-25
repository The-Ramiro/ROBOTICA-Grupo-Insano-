MODULE IMT342_EgmEnlace
    ! IRC5 / RobotWare 6. Importar como modulo adicional de T_ROB1.
    ! ROB_1 y default se verificaron en el controlador 120-101516.
    PERS string imtLinkDevice := "ROB_1";
    PERS string imtLinkProfile := "default";

    PROC IMT342_ProbarEnlace()
        VAR egmident linkId;
        VAR bool reserved := FALSE;

        IF imtLinkDevice = "" OR imtLinkProfile = "" THEN
            TPWrite "Completar dispositivo UDP y perfil EGM.";
            RETURN;
        ENDIF

        EGMGetId linkId;
        reserved := TRUE;
        EGMSetupUC ROB_1, linkId, imtLinkProfile, imtLinkDevice\Joint;
        EGMStreamStart linkId;
        TPWrite "IMT342: enviando posiciones EGM durante 5 s.";
        WaitTime 5;
        EGMStreamStop linkId;
        EGMReset linkId;
        reserved := FALSE;
        TPWrite "IMT342: prueba UDP terminada.";
    ERROR
        IF reserved THEN
            EGMReset linkId;
        ENDIF
        RAISE;
    ENDPROC
ENDMODULE
