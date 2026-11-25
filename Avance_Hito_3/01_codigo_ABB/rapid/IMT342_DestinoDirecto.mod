MODULE IMT342_DestinoDirecto
    ! ABB IRB 120-3/0.6, IRC5, T_ROB1, manual reducido, brida tool0.
    ! Escribir directoInicio y directoDestino antes de publicar directoOrden.
    ! Usar una solicitud a la vez y esperar directoHecho = directoOrden.
    LOCAL CONST num margen := 2;
    LOCAL CONST num toleranciaInicio := 0.05;
    LOCAL CONST num alturaMinima := 0;
    LOCAL CONST speeddata velocidadDirecta := [20,5,20,5];

    PERS jointtarget directoInicio := [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS jointtarget directoDestino := [[9E9,9E9,9E9,9E9,9E9,9E9],[9E9,9E9,9E9,9E9,9E9,9E9]];
    PERS num directoOrden := 0;
    PERS num directoHecho := 0;
    PERS num directoEstado := 0;
    PERS num directoActivo := 0;
    PERS num directoFin := 0;

    PROC ServidorDirecto()
        VAR num orden := 0;
        VAR jointtarget inicio;
        VAR jointtarget destino;

        directoActivo := 0;
        IF OpMode() <> OP_MAN_PROG THEN
            TPWrite "Seleccione manual reducido.";
            RETURN;
        ENDIF
        ! Descartar cualquier orden pendiente de una sesion anterior.
        directoFin := 0;
        directoEstado := 0;
        directoHecho := directoOrden;
        directoActivo := 1;
        TPWrite "Servidor directo activo. Esperando destino.";
        WHILE directoFin = 0 AND OpMode() = OP_MAN_PROG DO
            IF directoOrden <> directoHecho THEN
                orden := directoOrden;
                inicio := directoInicio;
                destino := directoDestino;
                directoEstado := 3;
                RevisarYPosicionar orden,inicio,destino;
                directoHecho := orden;
            ENDIF
            WaitTime 0.1;
        ENDWHILE
        directoActivo := 0;
        TPWrite "Servidor directo terminado.";
    ERROR
        directoEstado := -7;
        directoActivo := 0;
        directoHecho := orden;
        RAISE;
    ENDPROC

    LOCAL PROC RevisarYPosicionar(num orden, jointtarget inicio, jointtarget destino)
        VAR jointtarget actual;
        VAR jointtarget objetivo;
        VAR robtarget poseInicio;
        VAR robtarget poseDestino;

        actual := CJointT();
        IF DentroDeLimites(actual.robax) = FALSE OR DentroDeLimites(destino.robax) = FALSE THEN
            directoEstado := -2;
            RETURN;
        ENDIF
        IF CoincideInicio(actual.robax,inicio.robax) = FALSE THEN
            directoEstado := -3;
            RETURN;
        ENDIF
        IF SinEjesExternos(actual.extax) = FALSE OR SinEjesExternos(destino.extax) = FALSE THEN
            directoEstado := -6;
            RETURN;
        ENDIF
        objetivo := destino;
        objetivo.extax := actual.extax;
        ! Z de los extremos en wobj0, mm. Revisar aparte el recorrido completo.
        poseInicio := CalcRobT(actual,tool0\WObj:=wobj0);
        poseDestino := CalcRobT(objetivo,tool0\WObj:=wobj0);
        IF poseInicio.trans.z < alturaMinima OR poseDestino.trans.z < alturaMinima THEN
            directoEstado := -5;
            RETURN;
        ENDIF

        IF directoFin <> 0 OR directoOrden <> orden THEN
            directoEstado := -1;
            RETURN;
        ENDIF
        actual := CJointT();
        IF OpMode() <> OP_MAN_PROG OR CoincideInicio(actual.robax,inicio.robax) = FALSE OR
           SinEjesExternos(actual.extax) = FALSE THEN
            directoEstado := -3;
            RETURN;
        ENDIF
        poseInicio := CalcRobT(actual,tool0\WObj:=wobj0);
        IF poseInicio.trans.z < alturaMinima THEN
            directoEstado := -5;
            RETURN;
        ENDIF

        directoEstado := 1;
        MoveAbsJ objetivo,velocidadDirecta,fine,tool0\WObj:=wobj0;
        directoEstado := 2;
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
        RETURN Abs(actual.rax_1-inicio.rax_1) <= toleranciaInicio AND
               Abs(actual.rax_2-inicio.rax_2) <= toleranciaInicio AND
               Abs(actual.rax_3-inicio.rax_3) <= toleranciaInicio AND
               Abs(actual.rax_4-inicio.rax_4) <= toleranciaInicio AND
               Abs(actual.rax_5-inicio.rax_5) <= toleranciaInicio AND
               Abs(actual.rax_6-inicio.rax_6) <= toleranciaInicio;
    ENDFUNC

    LOCAL FUNC bool SinEjesExternos(extjoint ejes)
        RETURN ejes.eax_a >= 8E9 AND ejes.eax_b >= 8E9 AND ejes.eax_c >= 8E9 AND
               ejes.eax_d >= 8E9 AND ejes.eax_e >= 8E9 AND ejes.eax_f >= 8E9;
    ENDFUNC

ENDMODULE
