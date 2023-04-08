#pragma once

/**
* The endgame subsystem is responsible for deploying the endgame
*/
namespace endgame {

    /**
    * Deploys the endgame 
    */
    void deploy();

    /**
     * Deploys the left endgame
     */
    void deploy_left();

    /**
     * Deploys the right endgame
     */
    void deploy_right();
}