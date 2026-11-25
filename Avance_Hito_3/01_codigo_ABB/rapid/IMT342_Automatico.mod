MODULE IMT342_Automatico
    ! ABB IRB 120-3/0.6, IRC5, T_ROB1, modo automatico, brida tool0.
    ! Escribir autoInicio y autoDestino antes de publicar autoOrden.
    ! Usar una solicitud a la vez y esperar autoHecho = autoOrden.
    LOCAL CONST num margen := 2;
    LOCAL CONST num toleranciaInicio := 0.05;
    LOCAL CONST num toleranciaLlegada := 0.1;
    LOCAL CONST num toleranciaEstable := 0.02;
    LOCAL CONST num alturaMinima := 0;
    ! vmax usa la velocidad maxima configurada por ABB para este robot.
    ! El panel confirma speedratio = 0 antes de iniciar cada sesion.

    PERS jointtarget autoInicio := [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget autoDestino := [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS num autoOrden := 0;
    PERS num autoHecho := 0;
    PERS num autoEstado := 0;
    PERS num autoActivo := 0;
    PERS num autoFin := 0;
    PERS num autoSesion := 0;

    PERS num autoCantidad := 0;
    PERS num autoTramo := 0;
    PERS num autoUltimo := 0;
    PERS num autoErrorEje := 0;
    PERS num autoErrorDelta := 0;
    PERS num autoErrorRAPID := 0;
    PERS jointtarget autoRuta{20} := [
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]],
        [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]]
    ];

    PROC ServidorAutomatico()
        VAR num orden := 0;
        VAR jointtarget inicio;
        VAR jointtarget destino;
        VAR jointtarget puntos{20};
        VAR num cantidad;
        VAR bool valida;
        VAR robtarget pose;

        autoActivo := 0;
        IF OpMode() <> OP_AUTO THEN
            TPWrite "Seleccione modo automatico.";
            RETURN;
        ENDIF
        ! Descartar cualquier orden pendiente de una sesion anterior.
        autoFin := 0;
        autoEstado := 0;
        autoTramo := 0;
        autoHecho := autoOrden;
        autoSesion := (autoSesion MOD 1000000) + 1;
        autoActivo := 1;
        TPWrite "Servidor automatico activo. Esperando destino.";
        WHILE autoFin = 0 AND OpMode() = OP_AUTO DO
            IF autoOrden <> autoHecho THEN
                orden := autoOrden;
                inicio := autoInicio;
                destino := autoDestino;
                cantidad := autoCantidad;
                autoTramo := 0;
                autoUltimo := 0;
                autoErrorEje := 0;
                autoErrorDelta := 0;
                autoErrorRAPID := 0;
                autoEstado := 3;
                IF cantidad < 0 OR cantidad > 20 OR (cantidad MOD 1) <> 0 THEN
                    autoEstado := -8;
                ELSEIF cantidad = 0 THEN
                    autoTramo := 1;
                    RevisarYPosicionar orden,inicio,destino;
                    IF autoEstado = 2 THEN
                        autoUltimo := 1;
                    ENDIF
                ELSE
                    ! Copia local completa antes de empezar. Una orden por ruta.
                    FOR i FROM 1 TO 20 DO
                        puntos{i} := autoRuta{i};
                    ENDFOR
                    valida := TRUE;
                    FOR i FROM 1 TO cantidad DO
                        IF valida THEN
                            autoTramo := i;
                            IF DentroDeLimites(puntos{i}.robax) = FALSE OR
                               SinEjesExternos(puntos{i}.extax) = FALSE THEN
                                autoEstado := -2;
                                valida := FALSE;
                            ELSE
                                pose := CalcRobT(puntos{i},tool0\WObj:=wobj0);
                                IF pose.trans.z < alturaMinima THEN
                                    autoEstado := -5;
                                    valida := FALSE;
                                ENDIF
                            ENDIF
                        ENDIF
                    ENDFOR
                    IF valida THEN
                        FOR i FROM 1 TO cantidad DO
                            IF valida THEN
                                autoTramo := i;
                                RevisarYPosicionar orden,inicio,puntos{i};
                                IF autoEstado <> 2 THEN
                                    valida := FALSE;
                                ELSE
                                    autoUltimo := i;
                                    ! Usar la postura medida, ya estable y comprobada.
                                    inicio := CJointT();
                                ENDIF
                            ENDIF
                        ENDFOR
                    ENDIF
                ENDIF
                autoHecho := orden;
            ENDIF
            WaitTime 0.1;
        ENDWHILE
        autoActivo := 0;
        TPWrite "Servidor directo terminado.";
    ERROR
        autoErrorRAPID := ERRNO;
        autoEstado := -7;
        autoActivo := 0;
        autoHecho := orden;
        RAISE;
    ENDPROC

    LOCAL PROC RevisarYPosicionar(num orden, jointtarget inicio, jointtarget destino)
        VAR jointtarget actual;
        VAR jointtarget objetivo;
        VAR robtarget poseInicio;
        VAR robtarget poseDestino;
        VAR jointtarget previa;
        VAR num estables := 0;
        VAR bool llegada := FALSE;

        actual := CJointT();
        IF DentroDeLimites(actual.robax) = FALSE OR DentroDeLimites(destino.robax) = FALSE THEN
            autoEstado := -2;
            RETURN;
        ENDIF
        IF CoincideInicio(actual.robax,inicio.robax) = FALSE THEN
            GuardarDiferencia actual.robax,inicio.robax;
            autoEstado := -3;
            RETURN;
        ENDIF
        IF SinEjesExternos(actual.extax) = FALSE OR SinEjesExternos(destino.extax) = FALSE THEN
            autoEstado := -6;
            RETURN;
        ENDIF
        objetivo := destino;
        objetivo.extax := actual.extax;
        ! Z de los extremos en wobj0, mm. Revisar aparte el recorrido completo.
        poseInicio := CalcRobT(actual,tool0\WObj:=wobj0);
        poseDestino := CalcRobT(objetivo,tool0\WObj:=wobj0);
        IF poseInicio.trans.z < alturaMinima OR poseDestino.trans.z < alturaMinima THEN
            autoEstado := -5;
            RETURN;
        ENDIF

        IF autoFin <> 0 OR autoOrden <> orden THEN
            autoEstado := -1;
            RETURN;
        ENDIF
        actual := CJointT();
        IF OpMode() <> OP_AUTO THEN
            autoEstado := -4;
            RETURN;
        ENDIF
        IF CoincideInicio(actual.robax,inicio.robax) = FALSE THEN
            GuardarDiferencia actual.robax,inicio.robax;
            autoEstado := -3;
            RETURN;
        ENDIF
        IF SinEjesExternos(actual.extax) = FALSE THEN
            autoEstado := -6;
            RETURN;
        ENDIF
        poseInicio := CalcRobT(actual,tool0\WObj:=wobj0);
        IF poseInicio.trans.z < alturaMinima THEN
            autoEstado := -5;
            RETURN;
        ENDIF

        autoEstado := 1;
        estables := 0;
        llegada := FALSE;
        MoveAbsJ objetivo,vmax,fine,tool0\WObj:=wobj0;
        WaitRob \InPos;
        previa := CJointT();
        ! Confirmacion acotada: dos muestras estables, no un tiempo fijo a ciegas.
        FOR intento FROM 1 TO 60 DO
            IF llegada = FALSE THEN
                WaitTime 0.05;
                actual := CJointT();
                IF Coincide(actual.robax,objetivo.robax,toleranciaLlegada) AND
                   Coincide(actual.robax,previa.robax,toleranciaEstable) THEN
                    estables := estables + 1;
                ELSE
                    estables := 0;
                ENDIF
                previa := actual;
                llegada := estables >= 2;
            ENDIF
        ENDFOR
        IF llegada = FALSE THEN
            GuardarDiferencia actual.robax,objetivo.robax;
            autoEstado := -9;
            RETURN;
        ENDIF
        autoEstado := 2;
    ENDPROC

    LOCAL FUNC bool DentroDeLimites(robjoint q)
        RETURN q.rax_1 >= (-165+margen) AND q.rax_1 <= (165-margen) AND
               q.rax_2 >= (-110+margen) AND q.rax_2 <= (110-margen) AND
               q.rax_3 >= (-110+margen) AND q.rax_3 <= (70-margen) AND
               q.rax_4 >= (-160+margen) AND q.rax_4 <= (160-margen) AND
               q.rax_5 >= (-120+margen) AND q.rax_5 <= (120-margen) AND
               q.rax_6 >= (-400+margen) AND q.rax_6 <= (400-margen);
    ENDFUNC

    LOCAL FUNC bool CoincideInicio(robjoint actual, robjoint inicio)
        RETURN Coincide(actual,inicio,toleranciaInicio);
    ENDFUNC

    LOCAL FUNC bool Coincide(robjoint actual, robjoint esperado, num tolerancia)
        RETURN Abs(actual.rax_1-esperado.rax_1) <= tolerancia AND
               Abs(actual.rax_2-esperado.rax_2) <= tolerancia AND
               Abs(actual.rax_3-esperado.rax_3) <= tolerancia AND
               Abs(actual.rax_4-esperado.rax_4) <= tolerancia AND
               Abs(actual.rax_5-esperado.rax_5) <= tolerancia AND
               Abs(actual.rax_6-esperado.rax_6) <= tolerancia;
    ENDFUNC

    LOCAL PROC GuardarDiferencia(robjoint actual, robjoint esperado)
        VAR num diferencias{6};
        diferencias := [Abs(actual.rax_1-esperado.rax_1),Abs(actual.rax_2-esperado.rax_2),
                        Abs(actual.rax_3-esperado.rax_3),Abs(actual.rax_4-esperado.rax_4),
                        Abs(actual.rax_5-esperado.rax_5),Abs(actual.rax_6-esperado.rax_6)];
        autoErrorEje := 1;
        autoErrorDelta := diferencias{1};
        FOR eje FROM 2 TO 6 DO
            IF diferencias{eje} > autoErrorDelta THEN
                autoErrorEje := eje;
                autoErrorDelta := diferencias{eje};
            ENDIF
        ENDFOR
    ENDPROC

    LOCAL FUNC bool SinEjesExternos(extjoint ejes)
        RETURN ejes.eax_a >= 8E9 AND ejes.eax_b >= 8E9 AND ejes.eax_c >= 8E9 AND
               ejes.eax_d >= 8E9 AND ejes.eax_e >= 8E9 AND ejes.eax_f >= 8E9;
    ENDFUNC

ENDMODULE
