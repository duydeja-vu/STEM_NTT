void loop()
{
    if (nlr == 7)
    {
        found = true;
        for (int i = 0; i <= 2; i++)
        {
            Serial.print(dir[i]);
        }
        i = -1;
        j = 0;
        nlr = 0;
    }
    digitalWrite(10, LOW);
    digitalWrite(11, LOW);
    digitalWrite(12, LOW);
    digitalWrite(13, LOW);
    delay(30000);
    float fdist;
    float rdist;
    float ldist;
    // front, right, and left
    float fduration;
    float rduration;
    float lduration; // front, right, and left
    float fdur[5];
    float rdur[5];
    float ldur[5];

    float ldista[5];
    // For the front US sensor..
    for (int i = 0; i <= 4; i++)
    {
        digitalWrite(trigPinf, LOW);
        // Clearing the trigPin.
        delayMicroseconds(5);
        digitalWrite(trigPinf, HIGH);
        // Setting the trigPin HIGH for 10
        microseconds..sends some 8cycle sonics.delayMicroseconds(10);
        digitalWrite(trigPinf, LOW);
        fdur[i] = pulseIn(echoPinf, HIGH);
        travelled.
    }
    fduration = middleval(fdur);
    fdist = fduration * 0.0344 / 2;
    direction for (int i = 0; i <= 4; i++)
    {
        digitalWrite(trigPinr, LOW);
        delayMicroseconds(5);
        digitalWrite(trigPinr, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPinr, LOW);
        rdur[i] = pulseIn(echoPinr, HIGH);
    }
    rduration = middleval(rdur);
    rdist = rduration * 0.0344 / 2;

    for (int i = 0; i <= 4; i++)
    {
        digitalWrite(trigPinl, LOW);
        delayMicroseconds(5);
        digitalWrite(trigPinl, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPinl, LOW);
        ldur[i] = pulseIn(echoPinl, HIGH);
    }
    lduration = middleval(ldur);
    ldist = lduration * 0.0344 / 2;

    if ((fdist >= 125) || (rdist >= 150) || (ldist >= 400))
    {
        return;
    }

    if (rdist <= rthold)
        rsensor = true;
    if (ldist <= lthold)
        lsensor = true;
    if (fdist <= fthold)
        fsensor = true;
    // If left is closed-
    if ((lsensor == true))
    {
        // for a U-turn..
        if ((rsensor == true) && (fsensor == true))
        {
            j = j + 1;
            i = i + 1;
            dir[i] = 3;
            reduce(dir, i);
        }
        digitalWrite(10, HIGH);
        digitalWrite(11, LOW);
        digitalWrite(12, LOW);
        digitalWrite(13, HIGH);
        delay(2 * t);
        // If Front is open..
        else if (fsensor == false)
        {
            if ((rsensor == false) && (frontdist() >= 40))
                {
                    i = i + 1;
                    j = j + 1;
                    if ((found == true) & (dir[i] != 0))
                        else
                        {
                            if (found == false)
                            {
                                {
                                }
                               
                                rightturn();
                                return;
                                dir[i] = 0; 
                                reduce(dir, i);
                            }
                            timefr = tfr + 65 * nf;
                            nf = nf + 1;
                            stopit();
                            delay(1000);
                            for (int g = 1; g <= 10; g++)
                            {
                                gofront();
                                delay(timefr / 10);
                            }
                        }
                    stopit();
                    delay(1000);
                }
            else
            {
                gofront();
                delay(300);
            }
        }
        // Else moving forward .. only front is open.
        //for a right turn..
        else
        {
            i = i + 1;
            j = j + 1;
            dir[i] = 2;
            reduce(dir, i);
            float prevfdist = frontdist();
            while (abs(frontdist() - prevfdist) <= (prevfdist / 2) - 2)
            {
                gofront();
                delay(300);
                if (frontdist() <= 4.5)
                {
                    break;
                }
            }
            digitalWrite(10, HIGH);
            digitalWrite(11, LOW);
            digitalWrite(12, LOW);
            digitalWrite(13, HIGH);
            delay(t);
            float prevfrdist = frontdist();
            while (abs(frontdist() -
                       prevfrdist) <= 15.2)
            {
                gofront();
                delay(300);
                if (frontdist() <= 4.5)
                {
                    break;
                }
            }
        }
    }
    else
    {
        //for a left turn..
        i = i + 1;
        j = j + 1;
        if ((found == true) && (dir[i] != 1))
        {
            if ((dir[i] == 2) && (rightdist >= 10))
            {
                rightturn();
                return;
            }
            else if ((dir[i] == 0) && (fsensor == false))
            {
                frontturn();
                return;
            }
        }
        else
        {
            dir[i] = 1;
            // Left turn..nlr=nlr+1;
            reduce(dir, i); //calling reduce function to shorten the path
            dynamically..if path is not yet completed
            {
                gofront();
                delay(tlbef);
            }
            digitalWrite(10, LOW); // takes a left turn..
            digitalWrite(11, LOW);
            digitalWrite(12, HIGH);
            digitalWrite(13, LOW);
            delay(2 * t);
            for (int n = 1; n <= 8; n++)
            {
                gofront();
                delay(tlaf / 8);
            }
        }
        stopit();
        delay(1000);
    }
    delay(320);
}