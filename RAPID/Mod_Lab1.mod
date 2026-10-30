MODULE Mod_Lab1
    !***************************************************************
    ! Laboratorio 1 - Robotica Industrial UNAL (LabSIR) - 2026-II
    ! Integrantes: Juan y Carlos
    ! Decoracion automatica de torta con ABB IRB 140 y banda
    ! transportadora. Modulo unico: datos, programa principal,
    ! control de la celda y trayectorias de decoracion.
    !
    ! Programa principal de la celda:
    !   DI_01 -> Ciclo automatico: banda -> decoracion -> HOME -> banda
    !            DO_01 = 1 solo mientras se dibuja.
    !   DI_02 -> Pose de mantenimiento. DO_02 = 1 mientras el robot
    !            permanece en la pose (mientras DI_02 siga en 1).
    !   DI_03 -> Rutina "Farmear Aura" (baile). DO_03 = 1 durante
    !            toda la rutina, hasta llegar de nuevo a HOME.
    !
    ! La posicion de la torta se controla por TIEMPO (no hay sensor):
    !   tiempo = distancia / velocidad de la banda.
    ! VelBanda, DistEntrada y DistSalida se deben medir en el
    ! laboratorio; en RobotStudio deben coincidir con la velocidad
    ! del componente inteligente que mueve la torta.
    !
    ! Decoracion: JUAN Y CARLOS, bandera de Venezuela bajo JUAN,
    ! bandera de Chile bajo CARLOS y carita feliz al centro.
    ! Coordenadas en mm relativas a wobj_Torta:
    !   origen en la esquina superior lejana de la torta,
    !   X hacia el robot, Y a lo largo de la banda, Z hacia ABAJO
    !   (dentro de la torta). Por eso las alturas sobre la torta son
    !   negativas: zSeg = -15 esta 15 mm POR ENCIMA de la superficie.
    ! La herramienta trabaja con sus ejes paralelos a los del
    ! WorkObject (pRef con orientacion [1,0,0,0]).
    !
    ! Estructura del modulo:
    !   1. Datos: herramienta, WorkObject, HOME y decoracion
    !   2. Datos: banda, poses y velocidades de la celda
    !   3. Programa principal y rutinas de la celda (DI_01/02/03)
    !   4. Banda transportadora con enclavamientos
    !   5. Trayectorias de decoracion
    !***************************************************************

    !===============================================================
    ! 1. DATOS: HERRAMIENTA, WORKOBJECT, HOME Y DECORACION
    !===============================================================
    ! Altura de retiro entre trazos: 15 mm sobre la superficie (Z del wobj apunta hacia abajo)
    CONST num zSeg:=-15;
    ! Profundidad de trazo: 2 mm dentro de la superficie para comprimir el resorte
    CONST num zPen:=2;
    ! Velocidades y zonas (limites del enunciado: v100..v1000, max z10)
    CONST speeddata vTrazo:=v100;
    CONST speeddata vAire:=v400;
    CONST zonedata zTrazo:=z1;
    CONST zonedata zAire:=z5;

    ! Punto de referencia: origen de wobj_Torta con los ejes de la
    ! herramienta alineados con los del WorkObject (Z hacia la torta).
    CONST robtarget pRef:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    PERS tooldata tool_Marcador:=[TRUE,[[139.3,0,162.5],[0.906307787,0,0.422618262,0]],[0.1,[52,0,82],[1,0,0,0],0,0,0]];
    TASK PERS wobjdata wobj_Torta:=[FALSE,TRUE,"",[[698,20,183],[0.017452,0,0.999848,0]],[[0,0,0],[1,0,0,0]]];
    CONST jointtarget jHome:=[[0,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

    !===============================================================
    ! 2. DATOS: BANDA, POSES Y VELOCIDADES DE LA CELDA
    !===============================================================
    !---------------------------------------------------------------
    ! Parametros de la banda (AJUSTAR con medidas reales)
    !---------------------------------------------------------------
    ! Velocidad de la banda [mm/s]. En la simulacion = velocidad de
    ! los LinearMover del componente inteligente SC_Banda.
    PERS num VelBanda:=100;
    ! Distancia [mm] que recorre la torta desde el punto de partida
    ! hasta la estacion de decoracion.
    PERS num DistEntrada:=500;
    ! Distancia [mm] desde la estacion hasta el extremo final.
    PERS num DistSalida:=500;
    ! Tiempo de espera para cambiar el sentido de la banda [s]
    CONST num tCambioSentido:=0.5;
    ! Tiempo de asentamiento tras detener la banda [s]
    CONST num tAsentamiento:=0.5;
    ! TRUE: al terminar el ciclo DI_01, la banda regresa la torta al
    ! punto de partida en sentido BWD. Sirve para demostrar el
    ! funcionamiento bidireccional y repetir el ciclo. FALSE: el
    ! ciclo termina con la torta en el extremo final.
    PERS bool RetornoDemo:=FALSE;

    !---------------------------------------------------------------
    ! Posiciones articulares
    !---------------------------------------------------------------
    ! Pose de mantenimiento: robot girado 90 grados hacia el lado
    ! libre de la celda (lejos de la banda), brazo horizontal y
    ! flange a ~1.1 m del piso: comodo para cambiar el marcador.
    CONST jointtarget jMant:=[[90,0,0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    ! Pose central del baile: HOME con la muneca inclinada 30 grados
    ! (eje 5). Evita la singularidad de muneca de HOME (eje 5 = 0)
    ! durante los movimientos lineales del baile.
    CONST jointtarget jBaile:=[[0,0,0,0,30,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    ! Tolerancia para considerar que el robot esta en HOME [grados]
    CONST num TolHome:=1;

    !---------------------------------------------------------------
    ! Velocidades (dentro de v100..v1000) y zonas (max z10)
    !---------------------------------------------------------------
    CONST speeddata vMov:=v400;
    ! Baile: 300 mm/s del TCP y 120 grados/s de reorientacion
    CONST speeddata vBaile:=[300,120,5000,1000];
    ! Saludo (giro del marcador): reorientacion lenta, 60 grados/s
    CONST speeddata vSaludo:=[100,60,5000,1000];

    !---------------------------------------------------------------
    ! Seccion 4.3: coordenadas del sistema de referencia de la torta
    ! en el cuadrante x(+), y(-). Se usa el MISMO wobj_Torta y solo
    ! se cambia su posicion (uframe); las trayectorias no cambian.
    !---------------------------------------------------------------
    CONST pos posTortaYneg:=[575,-260,150];

    !===============================================================
    ! 3. PROGRAMA PRINCIPAL
    !===============================================================
    PROC main()
        Inicializar;
        WHILE TRUE DO
            ! Solo se atiende una rutina a la vez (prioridad 1-2-3) y
            ! solo si las otras dos entradas estan en 0.
            IF DI_01=1 AND DI_02=0 AND DI_03=0 THEN
                CicloDecoracion;
                EsperarLiberacion "DI_01";
            ELSEIF DI_02=1 AND DI_01=0 AND DI_03=0 THEN
                Mantenimiento;
            ELSEIF DI_03=1 AND DI_01=0 AND DI_02=0 THEN
                FarmearAura;
                EsperarLiberacion "DI_03";
            ENDIF
            WaitTime 0.1;
        ENDWHILE
    ENDPROC

    !---------------------------------------------------------------
    ! Estado inicial seguro: banda detenida, indicadores apagados y
    ! robot en HOME.
    !---------------------------------------------------------------
    PROC Inicializar()
        TPErase;
        TPWrite "LAB 1 - DECORACION DE TORTAS - LabSIR";
        ConveyorStop;
        Reset DO_01;
        Reset DO_02;
        Reset DO_03;
        ConfJ\On;
        ConfL\On;
        SingArea\Off;
        HomeRobot;
        TPWrite "Listo. DI_01: ciclo | DI_02: mantenimiento | DI_03: Farmear Aura";
    ENDPROC

    !---------------------------------------------------------------
    ! Espera a que el operador desactive la entrada que inicio la
    ! rutina, para que no se repita sola (deteccion de flanco).
    !---------------------------------------------------------------
    PROC EsperarLiberacion(string nombre)
        TPWrite "Rutina terminada. Desactive "+nombre;
        IF nombre="DI_01" THEN
            WaitUntil DI_01=0;
        ELSEIF nombre="DI_03" THEN
            WaitUntil DI_03=0;
        ENDIF
    ENDPROC

    !===============================================================
    ! DI_01 - CICLO AUTOMATICO DE TRANSPORTE Y DECORACION
    !===============================================================
    PROC CicloDecoracion()
        TPWrite "INICIO CICLO DE DECORACION";
        Reset DO_02;
        Reset DO_03;

        ! Etapa 1: el robot debe estar en HOME antes de mover la banda
        IF NOT RobotEnHome() THEN
            TPWrite "Robot fuera de HOME: regresando antes de iniciar";
            Ir_Home;
        ENDIF

        ! Etapa 1: transporte de la torta hacia el robot
        TPWrite "Etapa 1: transportando torta a la estacion";
        MoverBanda DistEntrada,TRUE;

        ! Etapa 2: decoracion (solo con la banda detenida)
        IF BandaEnMovimiento() THEN
            TPWrite "ERROR: la banda sigue en movimiento";
            ConveyorStop;
            RETURN;
        ENDIF
        TPWrite "Etapa 2: decorando";
        WaitTime 1;          ! espera extra por el drift fisico de la banda
        Set DO_01;
        Decorar wobj_Torta;
        Reset DO_01;

        ! Etapa 3: retiro de la zona de interaccion y regreso a HOME
        TPWrite "Etapa 3: regresando a HOME";
        Ir_Home;

        ! Etapa 4: transporte de la torta decorada al extremo final
        ! (solo si el robot esta en posicion segura)
        IF RobotEnHome() THEN
            TPWrite "Etapa 4: transportando torta decorada";
            MoverBanda DistSalida,TRUE;
            TPWrite "CICLO TERMINADO";
        ELSE
            TPWrite "ERROR: robot fuera de HOME, la banda no se mueve";
            RETURN;
        ENDIF

        ! Opcional: retorno de la torta al inicio con BWD
        IF RetornoDemo THEN
            TPWrite "Retorno de la torta al inicio (BWD)";
            MoverBanda DistEntrada+DistSalida,FALSE;
        ENDIF
    ERROR
        ! Manejo basico de errores: dejar la celda en estado seguro
        ConveyorStop;
        Reset DO_01;
        TPWrite "Error en el ciclo. ERRNO = "\Num:=ERRNO;
        RAISE;
    ENDPROC

    !===============================================================
    ! DI_02 - POSE DE MANTENIMIENTO
    !===============================================================
    PROC Mantenimiento()
        TPWrite "MODO MANTENIMIENTO";
        ConveyorStop;
        Reset DO_01;
        Reset DO_03;
        ! Salida segura: primero a HOME y luego a la pose
        HomeRobot;
        MoveAbsJ jMant\NoEOffs,vMov,fine,tool_Marcador;
        ! Robot en la pose de mantenimiento
        Set DO_02;
        TPWrite "Robot en mantenimiento. Desactive DI_02 para salir";
        WaitUntil DI_02=0;
        ! El robot abandona la pose: se apaga el indicador
        Reset DO_02;
        HomeRobot;
        TPWrite "Fin de mantenimiento";
    ENDPROC

    !===============================================================
    ! DI_03 - RUTINA "FARMEAR AURA" (baile, ~15 s)
    ! Movimientos MoveJ y MoveL principalmente en Y y Z del mundo
    ! (casi nada en X). Estructuras usadas: FOR, IF y procedimientos.
    !===============================================================
    PROC FarmearAura()
        VAR robtarget pCentro;
        VAR num i;
        VAR num lado;

        TPWrite "FARMEANDO AURA";
        ConveyorStop;
        Reset DO_01;
        Reset DO_02;
        Set DO_03;

        ! Postura de inicio del baile (fuera de singularidad)
        MoveAbsJ jBaile\NoEOffs,vMov,fine,tool_Marcador;
        pCentro:=CRobT(\Tool:=tool_Marcador\WObj:=wobj0);
        SingArea\Wrist;

        ! Paso 1 - "Se levanta" (MoveJ): sube 60 mm
        MoveJ Offs(pCentro,0,0,60),vBaile,z10,tool_Marcador\WObj:=wobj0;

        ! Paso 2 - Balanceo lateral (MoveJ): izquierda / derecha en Y
        ! con un pequeno "dip" en Z en cada paso.
        FOR i FROM 1 TO 4 DO
            IF i MOD 2=1 THEN
                lado:=1;
            ELSE
                lado:=-1;
            ENDIF
            MoveJ Offs(pCentro,0,lado*120,40),vBaile,z10,tool_Marcador\WObj:=wobj0;
            MoveJ Offs(pCentro,0,0,-20),vBaile,z10,tool_Marcador\WObj:=wobj0;
        ENDFOR

        ! Paso 3 - Rebote vertical (MoveL): arriba / abajo en Z
        FOR i FROM 1 TO 3 DO
            MoveL Offs(pCentro,0,0,80),vBaile,z10,tool_Marcador\WObj:=wobj0;
            MoveL Offs(pCentro,0,0,-40),vBaile,z10,tool_Marcador\WObj:=wobj0;
        ENDFOR

        ! Paso 4 - Rombo en el plano YZ (MoveL)
        MoveL Offs(pCentro,0,100,20),vBaile,z10,tool_Marcador\WObj:=wobj0;
        MoveL Offs(pCentro,0,0,110),vBaile,z10,tool_Marcador\WObj:=wobj0;
        MoveL Offs(pCentro,0,-100,20),vBaile,z10,tool_Marcador\WObj:=wobj0;
        MoveL Offs(pCentro,0,0,-50),vBaile,z10,tool_Marcador\WObj:=wobj0;
        MoveL Offs(pCentro,0,100,20),vBaile,z10,tool_Marcador\WObj:=wobj0;

        ! Paso 5 - Saludo: gira el marcador sobre su propio eje
        MoveL pCentro,vBaile,fine,tool_Marcador\WObj:=wobj0;
        Saludo 2;

        ! Final: regreso a HOME y fin del indicador
        SingArea\Off;
        HomeRobot;
        Reset DO_03;
        TPWrite "AURA FARMEADA";
    ENDPROC

    ! Gira la herramienta +/-25 grados alrededor de su eje Z
    ! (RelTool), "veces" repeticiones.
    PROC Saludo(num veces)
        VAR robtarget p0;
        VAR num k;
        p0:=CRobT(\Tool:=tool_Marcador\WObj:=wobj0);
        FOR k FROM 1 TO veces DO
            MoveL RelTool(p0,0,0,0\Rz:=25),vSaludo,z10,tool_Marcador\WObj:=wobj0;
            MoveL RelTool(p0,0,0,0\Rz:=-25),vSaludo,z10,tool_Marcador\WObj:=wobj0;
        ENDFOR
        MoveL p0,vSaludo,fine,tool_Marcador\WObj:=wobj0;
    ENDPROC

    !===============================================================
    ! HOME
    !===============================================================
    ! Movimiento directo por ejes a HOME (se usa cuando el robot ya
    ! esta en una zona libre: inicio, mantenimiento, baile).
    ! Para salir de la torta se usa Ir_Home, que primero sube 50 mm.
    PROC HomeRobot()
        TPWrite "Regresando a HOME";
        MoveAbsJ jHome\NoEOffs,vMov,fine,tool_Marcador;
    ENDPROC

    ! TRUE si todos los ejes estan a menos de TolHome de jHome
    FUNC bool RobotEnHome()
        VAR jointtarget jAct;
        jAct:=CJointT();
        RETURN Abs(jAct.robax.rax_1-jHome.robax.rax_1)<TolHome
            AND Abs(jAct.robax.rax_2-jHome.robax.rax_2)<TolHome
            AND Abs(jAct.robax.rax_3-jHome.robax.rax_3)<TolHome
            AND Abs(jAct.robax.rax_4-jHome.robax.rax_4)<TolHome
            AND Abs(jAct.robax.rax_5-jHome.robax.rax_5)<TolHome
            AND Abs(jAct.robax.rax_6-jHome.robax.rax_6)<TolHome;
    ENDFUNC

    !===============================================================
    ! VERIFICACION DE ALCANZABILIDAD (basada en el Ejemplo 2)
    !===============================================================
    ! Comprueba con CalcJointT que las 4 esquinas de la zona de
    ! dibujo (a la altura de seguridad) son alcanzables.
    FUNC bool TortaAlcanzable(PERS wobjdata wobj)
        VAR jointtarget jTmp;
        VAR bool alcanzable:=TRUE;
        ! Igual que en Decorar: el robot escoge la configuracion mas
        ! cercana (pRef no fija configuracion de ejes)
        ConfJ\Off;
        ConfL\Off;
        jTmp:=CalcJointT(Offs(pRef,78,32,zSeg),tool_Marcador\WObj:=wobj);
        jTmp:=CalcJointT(Offs(pRef,78,211,zSeg),tool_Marcador\WObj:=wobj);
        jTmp:=CalcJointT(Offs(pRef,160,32,zSeg),tool_Marcador\WObj:=wobj);
        jTmp:=CalcJointT(Offs(pRef,160,211,zSeg),tool_Marcador\WObj:=wobj);
        ConfJ\On;
        ConfL\On;
        RETURN alcanzable;
    ERROR
        IF ERRNO=ERR_ROBLIMIT THEN
            alcanzable:=FALSE;
            TRYNEXT;
        ENDIF
    ENDFUNC

    ! Rutina opcional de verificacion (basada en el Ejemplo 2 de la
    ! guia). NO hace parte del ciclo: se ejecuta aparte, poniendo el
    ! puntero de programa aqui, y solo informa en el FlexPendant.
    ! Nota: CalcJointT usa la configuracion de ejes escrita en pRef
    ! ([0,0,0,0]); si el robot alcanza los puntos con otra
    ! configuracion puede reportar "no alcanzable" aunque el
    ! movimiento real (con ConfL\Off) si funcione.
    PROC VerificarAlcance()
        IF TortaAlcanzable(wobj_Torta) THEN
            TPWrite "Esquinas de la decoracion: alcanzables";
        ELSE
            TPWrite "Esquinas de la decoracion: revisar configuracion/alcance";
        ENDIF
    ENDPROC

    !===============================================================
    ! 4. CONTROL DE LA BANDA CON ENCLAVAMIENTOS (seccion 4.5)
    !===============================================================
    ! Mueve la banda una distancia [mm] en el sentido indicado y la
    ! detiene. El tiempo se calcula como distancia / VelBanda.
    PROC MoverBanda(num distancia,bool adelante)
        VAR num tMov;
        ! La banda solo se mueve con el robot en posicion segura
        IF NOT RobotEnHome() THEN
            TPWrite "Banda bloqueada: el robot no esta en HOME";
            RETURN;
        ENDIF
        tMov:=distancia/VelBanda;
        TPWrite "Banda en movimiento durante [s]: "\Num:=tMov;
        IF adelante THEN
            ConveyorForward;
        ELSE
            ConveyorBackward;
        ENDIF
        WaitTime tMov;
        ConveyorStop;
        ! Espera para garantizar la detencion completa
        WaitTime tAsentamiento;
    ENDPROC

    ! Avance: FWD=1, BWD=0. Si iba hacia atras, primero se detiene.
    PROC ConveyorForward()
        IF DOutput(BWD_Conveyor)=1 THEN
            ConveyorStop;
            WaitTime tCambioSentido;
        ENDIF
        Reset BWD_Conveyor;
        WaitTime 0.2;
        ! Enclavamiento: nunca FWD y BWD al mismo tiempo
        IF DOutput(BWD_Conveyor)=0 THEN
            Set FWD_Conveyor;
        ELSE
            ConveyorStop;
            TPWrite "ENCLAVAMIENTO: BWD activo, FWD no se activa";
        ENDIF
    ENDPROC

    ! Retroceso: FWD=0, BWD=1. Si iba hacia adelante, primero se detiene.
    PROC ConveyorBackward()
        IF DOutput(FWD_Conveyor)=1 THEN
            ConveyorStop;
            WaitTime tCambioSentido;
        ENDIF
        Reset FWD_Conveyor;
        WaitTime 0.2;
        IF DOutput(FWD_Conveyor)=0 THEN
            Set BWD_Conveyor;
        ELSE
            ConveyorStop;
            TPWrite "ENCLAVAMIENTO: FWD activo, BWD no se activa";
        ENDIF
    ENDPROC

    ! Banda detenida: FWD=0, BWD=0
    PROC ConveyorStop()
        Reset FWD_Conveyor;
        Reset BWD_Conveyor;
    ENDPROC

    FUNC bool BandaEnMovimiento()
        RETURN DOutput(FWD_Conveyor)=1 OR DOutput(BWD_Conveyor)=1;
    ENDFUNC

    !===============================================================
    ! DEMOSTRACION SECCION 4.3 (no asociada a una entrada digital)
    ! Ejecutar poniendo el puntero de programa en esta rutina.
    ! Reutiliza exactamente las mismas trayectorias: solo se modifican
    ! las coordenadas del sistema de referencia wobj_Torta (uframe)
    ! para dibujar en el cuadrante x(+), y(-). Al final se restauran
    ! las coordenadas originales de la estacion de decoracion.
    !===============================================================
    PROC Demo_CambioCoordenadas()
        VAR pos posOriginal;
        posOriginal:=wobj_Torta.uframe.trans;
        TPWrite "Decoracion con wobj_Torta trasladado a x(+), y(-)";
        ConveyorStop;
        HomeRobot;
        ! Unico cambio: coordenadas del sistema de referencia
        wobj_Torta.uframe.trans:=posTortaYneg;
        Decorar wobj_Torta;
        Ir_Home;
        ! Restaurar la posicion original (estacion sobre la banda)
        wobj_Torta.uframe.trans:=posOriginal;
        TPWrite "wobj_Torta restaurado a la estacion de decoracion";
    ERROR
        ! Si algo falla, siempre se restauran las coordenadas
        wobj_Torta.uframe.trans:=posOriginal;
        ConveyorStop;
        RAISE;
    ENDPROC
    !===============================================================
    ! 5. TRAYECTORIAS DE DECORACION
    !===============================================================
    PROC Decorar(PERS wobjdata wobj)
        ! Sin control de configuracion: el robot escoge la mas cercana
        ! en cada punto (pRef no fija configuracion de ejes).
        ConfJ\Off;
        ConfL\Off;
        MoveJ Offs(pRef,120,120,-80),vAire,z10,tool_Marcador\WObj:=wobj;
        Dib_JUAN wobj;
        Dib_Y wobj;
        Dib_CARLOS wobj;
        Dib_BANDERA_VEN wobj;
        Dib_BANDERA_CHI wobj;
        Dib_CARITA wobj;
        MoveL Offs(pRef,120,120,-80),vAire,z10,tool_Marcador\WObj:=wobj;
        ConfJ\On;
        ConfL\On;
        ! El regreso a HOME NO se hace aqui: lo hace CicloDecoracion
        ! despues de apagar DO_01, para que DO_01 quede activa
        ! exclusivamente durante las trayectorias.
    ENDPROC

    ! Rutina exclusiva para llevar el robot a HOME desde cualquier
    ! posicion: primero sube el TCP 50 mm en vertical (Z del mundo)
    ! para separarse de la torta y luego va a HOME por ejes.
    PROC Ir_Home()
        VAR robtarget pActual;
        ! Lee la posicion actual del TCP (punta del marcador) en el mundo
        pActual:=CRobT(\Tool:=tool_Marcador\WObj:=wobj0);
        ! Retiro vertical de seguridad: 50 mm hacia arriba
        MoveL Offs(pActual,0,0,50),vAire,fine,tool_Marcador\WObj:=wobj0;
        ! Movimiento por ejes a HOME (todos los ejes en 0 grados)
        MoveAbsJ jHome\NoEOffs,vAire,fine,tool_Marcador;
    ENDPROC

    ! Nombre JUAN (letras de 22 mm de alto)
    PROC Dib_JUAN(PERS wobjdata wobj)
        ! trazo 1
        MoveL Offs(pRef,78,205.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,205.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,197.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,94.5,197.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,100,203,zPen),Offs(pRef,94.5,208.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,94.5,208.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 2
        MoveL Offs(pRef,78,193.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,193.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,94.5,193.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,100,188,zPen),Offs(pRef,94.5,182.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,182.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,182.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 3
        MoveL Offs(pRef,100,178.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,178.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,173,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,167.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,167.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 4
        MoveL Offs(pRef,92,176.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,92,176.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,92,169.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,92,169.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 5
        MoveL Offs(pRef,100,163.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,163.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,163.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,152.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,152.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,152.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
    ENDPROC

    ! Letra Y entre los dos nombres
    PROC Dib_Y(PERS wobjdata wobj)
        ! trazo 1
        MoveL Offs(pRef,78,140.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,140.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,89,135,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,135,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,135,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 2
        MoveL Offs(pRef,78,129.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,129.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,89,135,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,89,135,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
    ENDPROC

    ! Nombre CARLOS (letras de 22 mm de alto)
    PROC Dib_CARLOS(PERS wobjdata wobj)
        ! trazo 1
        MoveL Offs(pRef,80.75,107.24,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,80.75,107.24,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,78,112,zPen),Offs(pRef,83.5,117.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,94.5,117.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,100,112,zPen),Offs(pRef,97.25,107.24,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,97.25,107.24,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 2
        MoveL Offs(pRef,100,102.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,102.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,97,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,91.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,91.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 3
        MoveL Offs(pRef,92,100.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,92,100.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,92,93.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,92,93.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 4
        MoveL Offs(pRef,100,87.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,87.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,87.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,82,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,83.5,76.5,zPen),Offs(pRef,89,82,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,89,87.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,89,87.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 5
        MoveL Offs(pRef,89,83.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,89,83.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,76.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,76.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 6
        MoveL Offs(pRef,78,72.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,78,72.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,72.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,61.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,100,61.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 7
        MoveL Offs(pRef,83.5,57.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,83.5,57.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,94.5,57.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,100,52,zPen),Offs(pRef,94.5,46.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,83.5,46.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,78,52,zPen),Offs(pRef,83.5,57.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,83.5,57.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 8
        MoveL Offs(pRef,80.75,32.24,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,80.75,32.24,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,78,37,zPen),Offs(pRef,83.5,42.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,87.39,40.89,zPen),Offs(pRef,89,37,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,94.5,31.5,zPen),Offs(pRef,100,37,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,99.26,39.75,zPen),Offs(pRef,97.25,41.76,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,97.25,41.76,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
    ENDPROC

    ! Bandera de Venezuela 60x40 mm: contorno, 2 franjas y 8 estrellas como puntos
    PROC Dib_BANDERA_VEN(PERS wobjdata wobj)
        ! trazo 1
        MoveL Offs(pRef,160,210.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,210.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,150.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,120,150.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,120,210.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,210.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,210.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 2
        MoveL Offs(pRef,146.67,150.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,146.67,150.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,146.67,210.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,146.67,210.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 3
        MoveL Offs(pRef,133.33,210.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,133.33,210.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,133.33,150.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,133.33,150.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,142.17,189.19,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,142.17,189.19,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,142.17,189.19,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,139.16,187.74,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,139.16,187.74,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,139.16,187.74,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,136.88,185.29,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,136.88,185.29,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,136.88,185.29,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.66,182.17,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.66,182.17,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.66,182.17,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.66,178.83,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.66,178.83,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.66,178.83,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,136.88,175.71,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,136.88,175.71,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,136.88,175.71,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,139.16,173.26,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,139.16,173.26,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,139.16,173.26,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,142.17,171.81,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,142.17,171.81,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,142.17,171.81,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
    ENDPROC

    ! Bandera de Chile 60x40 mm: contorno, franja, cuartel y estrella de 5 puntas
    PROC Dib_BANDERA_CHI(PERS wobjdata wobj)
        ! trazo 1
        MoveL Offs(pRef,160,104.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,104.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,44.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,120,44.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,120,104.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,104.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,160,104.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 2
        MoveL Offs(pRef,140,104.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,140,104.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,140,44.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,140,44.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 3
        MoveL Offs(pRef,140,84.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,140,84.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,120,84.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,120,84.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 4
        MoveL Offs(pRef,123.5,94.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,123.5,94.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.26,98.32,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,127.99,88.32,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,127.99,100.68,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,135.26,90.68,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,123.5,94.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,123.5,94.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
    ENDPROC

    ! Carita feliz: circulo de 28 mm de diametro, ojos y sonrisa
    PROC Dib_CARITA(PERS wobjdata wobj)
        ! trazo 1
        MoveL Offs(pRef,140,141.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,140,141.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,126,127.5,zPen),Offs(pRef,140,113.5,zPen),vTrazo,zTrazo,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,154,127.5,zPen),Offs(pRef,140,141.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,140,141.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 2
        MoveL Offs(pRef,137,132.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,137,132.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,133,132.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,133,132.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 3
        MoveL Offs(pRef,137,122.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,137,122.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,133,122.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,133,122.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        ! trazo 4
        MoveL Offs(pRef,143,135.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,143,135.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveC Offs(pRef,149,127.5,zPen),Offs(pRef,143,119.5,zPen),vTrazo,fine,tool_Marcador\WObj:=wobj;
        MoveL Offs(pRef,143,119.5,zSeg),vAire,zAire,tool_Marcador\WObj:=wobj;
    ENDPROC

    ! Prueba rapida en simulacion (no hace parte del main final)
    PROC Prueba_Decoracion()
        MoveAbsJ jHome\NoEOffs,vAire,fine,tool_Marcador;
        Decorar wobj_Torta;
        Ir_Home;
    ENDPROC
ENDMODULE
