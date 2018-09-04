% Task 2

%% Init 
load('TestTrack.mat');
Nobs = 25; %10-25
wins = 0;
num_trials = 10;
%%
for i=1:num_trials
    Xobs = generateRandomObstacles(Nobs,TestTrack);

    %% Actually Call the function
    tic
    [sol_2] = ROB599_ControlsProject_part2_Team16(TestTrack,Xobs);
    elapsed = toc
    
    % Must take less than 5 minutes (300 seconds)
    if(elapsed > 300)
        continue;
    end
    
    %% Forward Integrate Using Official Function 
    x = forwardIntegrateControlInput(sol_2);

    %% Check Trajectory
    failed = checkTrajectory(x,sol_2,Xobs);
    if(failed)
        %disp(' YOU SUCK!! GO HOME!!!');
    else
        wins = wins + 1;
        %disp(' YOU WIN! :)');
    end
end

disp('Total Trials = ');
disp(num_trials);

disp('Wins = ');
disp(wins);

%% ... and then read Ke$ha lyrics
% on the clock
% Cuz the party dont stop

% Oh oo-Oh oo-Oh OH
% Oh oo-Oh oo-Oh OH

%%
%{
Wake up in the morning feelin' like P Diddy (hey, what up, girl?)
Grab my glasses, I'm out the door, I'm gonna hit this city (let's go)
Before I leave, brush my teeth with a bottle of Jack
'Cause when I leave for the night, I ain't coming back

[Pre-Chorus 1]
I'm talkin' pedicure on our toes, toes
Tryin' on all our clothes, clothes
Boys blowin' up our phones, phones
Drop-toppin', playin' our favorite CDs
Pullin' up to the parties
Tryna get a little bit tipsy

[Chorus]
Don't stop, make it pop
DJ, blow my speakers up
Tonight, I'ma fight
Till we see the sunlight
Tick tock on the clock
But the party don't stop, no
Oh, whoa, whoa, oh
Oh, whoa, whoa, oh
Don't stop, make it pop
DJ, blow my speakers up
Tonight, I'ma fight
Till we see the sunlight
Tick tock on the clock
But the party don't stop, no
Oh, whoa, whoa, oh
Oh, whoa, whoa, oh

[Verse 2]
Ain't got a care in world but got plenty of beer
Ain't got no money in my pocket but I'm already here
And now the dudes are linin' up 'cause they hear we got swagger
But we kick 'em to the curb unless they look like Mick Jagger

[Pre-Chorus 2]
I'm talkin' about errbody gettin' crunk, crunk
Boys tryin' to touch my junk, junk
Gonna smack him if he gettin' too drunk, drunk
Now, now we go until they kick us out, out
Or the police shut us down, down
Police shut us down, down
Po-po shut us—

[Chorus]
Don't stop, make it pop
DJ, blow my speakers up
Tonight, I'ma fight
Till we see the sunlight
Tick tock on the clock
But the party don't stop, no
Oh, whoa, whoa, oh
Oh, whoa, whoa, oh
Don't stop, make it pop
DJ, blow my speakers up
Tonight, I'ma fight
Till we see the sunlight
Tick tock on the clock
But the party don't stop, no
Oh, whoa, whoa, oh
Oh, whoa, whoa, oh

[Bridge]
(DJ) You build me up, you break me down
My heart, it pounds, yeah, you got me
With my hands up, you got me now
You got that sound, yeah, you got me
(DJ) You build me up, you break me down
My heart, it pounds, yeah, you got me
With my hands up, put your hands up
Put your hands up

[Break]
Now, the party don't start till I walk in

[Chorus]
Don't stop, make it pop
DJ, blow my speakers up
Tonight, I'ma fight
Till we see the sunlight
Tick tock on the clock
But the party don't stop, no
Oh, whoa, whoa, oh
Oh, whoa, whoa, oh
Don't stop, make it pop
DJ, blow my speakers up
Tonight, I'ma fight
Till we see the sunlight
Tick tock on the clock
But the party don't stop, no
Oh, whoa, whoa, oh
Oh, whoa, whoa, oh
%}