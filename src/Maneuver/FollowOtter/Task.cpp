#include <DUNE/DUNE.hpp>
#include <iostream>
#include <algorithm>
namespace Maneuver
{
    namespace FollowOtter
    {
        using DUNE_NAMESPACES;

        struct Arguments
        {
            double loiter_radius;
            double timeout;
            bool announce_active;
            bool remote_active;
            double min_displace;
            double heading_cooldown;
            double safe_distance;
            bool   anti_collision;
            float kp;
            float ki;
            float kd;
            float desired_distance;

        };

        struct Task: public DUNE::Maneuvers::Maneuver
        {
            //! RPM PID controller  // this one to control the speed of the slave to keep the distance with the master fixed
            DiscretePID m_rpm_pid;
            //! bearing controller  // this one to control the speed of the slave to keep the distance with the master fixed
            DiscretePID m_bearing_pid;
            //! Control Parcels for meters per second controller
            IMC::ControlParcel m_parcel_mps;
            //! Variable to save the maneuver's data
            IMC::FollowSystem m_maneuver;
            //! Vehicle's Estimated State
            IMC::EstimatedState m_estate;  // for the slave vehicle
            //! Desired path to be thrown
            IMC::DesiredPath m_path;
            //! Remote State computed heading's timestamp, for evaluating the best heading to be used
            Counter<double> m_heading_timestamp;
            //! this variable will hold the value of the heading computed when using the announce method instead of the remote state.
            double m_remote_heading;
            //! the start time of the maneuver measured at consume maneuver
            double m_start_time;
            //! the last Clock::get() when the neighbor system's position was updated
            Counter<double> m_last_update;
            //! variable to hold the last known bearing
            double m_last_known_bearing;
            //! variable that will hold the last known latitude
            double m_last_known_lat;
            //! variable that will hold the last known longitude
            double m_last_known_lon;
            //! variable that will hold the last known latitude for the second slave
            double s_last_known_lat;
            //! variable that will hold the last known longitude for the second slave
            double s_last_known_lon;
            //! Time of last estimated state message.
            Delta m_delta;
            //! vehicle latitude and longitude
            double v_last_known_lat;
            //! variable that will hold the last known longitude for the second slave
            double v_last_known_lon;
            //! is it the first time consume announce is being ran?
            bool m_first_announce;
            //! this boolean tells us if we have an estimated state already
            bool m_has_estimated_state;
            //! this dp plan represents the master current plan
            IMC::PlanDB Master_Plan_Dp;
            //! thdisplay.push_back(r);is dp plan represents the Slave current plan
            IMC::PlanDB Slave_Plan_Dp;
            //! vector for all master sent estimated states
            std::vector<IMC::EstimatedState> master_estimated_states;
            //! the second slave estimated state
            IMC::EstimatedState m2_estate;
            double r2;
            // vector to be displayed
            // std::vector<double> display;
            
            //! Task Arguments
            Arguments m_args;

            Task(const std::string& name, Tasks::Context& ctx):
                    DUNE::Maneuvers::Maneuver(name, ctx),
                    m_last_known_bearing(0.0),
                    m_last_known_lat(0.0),
                    m_last_known_lon(0.0),
                    m_first_announce(true),
                    m_has_estimated_state(false)
            {
                param("Loitering Radius", m_args.loiter_radius)       // note these default values will be overwritten by the configuration files 
                        .defaultValue("-1.0")                         // in case we left the values empty then the parameters will be set to the default vlaues
                        .units(Units::Meter)
                        .description("Radius of the loiter when waiting for new waypoint");

                param("Timeout", m_args.timeout)
                        .defaultValue("60.0")
                        .units(Units::Second)
                        .description("Maneuver timeout");

                param("Using Announce", m_args.announce_active)
                        .defaultValue("false")
                        .description("Using announce to track system");

                param("Using RemoteState", m_args.remote_active)
                        .defaultValue("false")
                        .description("Using remote state for tracking system");

                param("Min Displace", m_args.min_displace)
                        .defaultValue("2.0")
                        .units(Units::Meter)
                        .description("Minimum target displacement for computing new heading");

                param("Heading Cooldown", m_args.heading_cooldown)
                        .defaultValue("20.0")
                        .units(Units::Second)
                        .description("");

                param("Minimum Safe Distance", m_args.safe_distance)
                        .defaultValue("15.0")
                        .units(Units::Meter)
                        .description("Minimum safe distance to target system");


                param("anti collision", m_args.anti_collision)
                        .defaultValue("true")
                        .description("avoid collision between the slave vehicles");


                param("kp", m_args.kp)                         //set the propotional gain
                        .defaultValue("1.0")
                        .description("proportional gain");


                param("kd", m_args.kd)                       // set the derivitive gain
                        .defaultValue("0.0525")
                        .description("Derivative gain");

                param("ki", m_args.ki)                      // the integeral gain 
                        .defaultValue("0.02")
                        .description("Integral gain");


                param("desired_distance", m_args.desired_distance)  //this will determine the formation shape as it set the desired x-offset
                        .defaultValue("15")
                        .description("desired distance");



                bindToManeuver<Task, IMC::FollowSystem>();
                bind<IMC::RemoteState>(this);
                bind<IMC::EstimatedState>(this, true); // consume even if inactive
                bind<IMC::Announce>(this);
            }

            void setup(void){   //this method to setup our pid controllers and its parameters 
                m_rpm_pid.setProportionalGain(m_args.kp);
                m_rpm_pid.setDerivativeGain(m_args.kd);
                m_rpm_pid.setIntegralGain(m_args.ki);
                m_rpm_pid.setOutputLimits(0.4, 4.0);
                m_rpm_pid.setIntegralLimits(1.5);
                m_rpm_pid.enableParcels(this, &m_parcel_mps);
                m_bearing_pid.setProportionalGain(m_args.kp);
                m_bearing_pid.setDerivativeGain(m_args.kd);
                m_bearing_pid.setIntegralGain(m_args.ki);
                m_bearing_pid.setOutputLimits(0.4, 4.0);
                m_bearing_pid.setIntegralLimits(0.5);
            }

            void
            onUpdateParameters(void)
            {
                if (paramChanged(m_args.timeout))
                    m_last_update.setTop(m_args.timeout);

                if (paramChanged(m_args.heading_cooldown))
                    m_heading_timestamp.setTop(m_args.heading_cooldown);
            }

            void
            onManeuverDeactivation(void)
            {
                m_first_announce = true;
                m_has_estimated_state = false;
            }

            void
            consume(const IMC::EstimatedState* msg)
            {
                if (msg->getSource() != getSystemId()){
                    if(msg->getSource()==m_maneuver.system ){
                        master_estimated_states.push_back(*msg);

                    }
                    else{

                        m2_estate = *msg;
                    }

                    return;}

                // do not do a thing if the announcement method is not active
                if (!m_args.announce_active)
                    return;

                m_estate = *msg;                // our slave estimated state
                m_has_estimated_state = true;   // set the boolean estimated state
                
            }

            void
            consume(const IMC::FollowSystem* maneuver)
            {
                enableMovement(false);
                setup();
                m_maneuver = *maneuver;
                m_heading_timestamp.reset();
                m_start_time = Clock::get();
                // Initialize the variable last update to the beginning of the maneuver
                m_last_update.reset();

                debug("loitering radius is %0.2f meters", m_args.loiter_radius);
                debug("offsets are %0.2f %0.2f %0.2f", m_maneuver.x, m_maneuver.y, m_maneuver.z);
                debug("speed is %0.2f units %d", m_maneuver.speed, (int)m_maneuver.speed_units);
            }

            void
            consume(const IMC::RemoteState* rs)
            {
                // Not the vehicle we are following or remote method is inactive
                if (rs->getSource() != m_maneuver.system || !m_args.remote_active)
                    return;

                // update the variable last update
                m_last_update.reset();

                // update the variable last heading update
                m_heading_timestamp.reset();

                if (!checkSafety(rs->lat, rs->lon))
                {
                    // leave this consume function but first update "last" variables
                    m_last_known_lat = rs->lat;
                    m_last_known_lon = rs->lon;
                    m_remote_heading = rs->psi;
                    enableMovement(false);
                    return;
                }

                enableMovement(true);

                // change the variable m_path according to the offsets in m_maneuver
                computeNEDOffsets(rs->lat, rs->lon, rs->depth, rs->psi);

                m_path.lradius = m_args.loiter_radius;
                m_path.flags = IMC::DesiredPath::FL_DIRECT;

                m_path.speed = m_maneuver.speed;
                m_path.speed_units = m_maneuver.speed_units;

                dispatch(m_path);

                m_last_known_lat = rs->lat;
                m_last_known_lon = rs->lon;
                m_remote_heading = rs->psi;

                trace("system being pursued has heading: %0.2f", rs->psi);
                trace("this is %0.4f seconds past the maneuver's initial time", Clock::get() - m_start_time);
                trace("remote data: lat %0.5f, lon %0.5f, depth %d, timestamp %0.4f", rs->lat, rs->lon, rs->depth, rs->getTimeStamp());
                trace("thrown waypoint: lat %0.5f %0.5f %0.2f", m_path.end_lat, m_path.end_lon, m_path.end_z);
            }
            void
            consume(const IMC::Announce* msg)
            {
                typedef std::numeric_limits< double > dbl;
                std::cout.precision(dbl::max_digits10);

                // Not the vehicle we are following or the announcement method is inactive
                if (msg->getSource() != m_maneuver.system || !m_args.announce_active) {
                    if (msg->getSource() != getSystemId() and
                        m_args.anti_collision) {  //! that means we received a message from the other slave


                        if (msg->getSource() == 10256 or msg->getSource() == 10259) {
                            s_last_known_lat = msg->lat;
                            s_last_known_lon = msg->lon;
                        }
                    }
                    else {
                        v_last_known_lat = msg->lat;
                        v_last_known_lon = msg->lon;
                    }
                    return;
                }

                // update the variable last update
                m_last_update.reset();

                // if present location is unsafe, then
                if (!checkSafety(msg->lat, msg->lon) )
                {

                    // leave this consume function but first update "last" variables
                    m_last_known_lat = msg->lat;
                    m_last_known_lon = msg->lon;
                    enableMovement(false);
                    return;
                }

                if( (!checkSafety2() )and m_args.anti_collision){
                    std::cout<<"distance violation between the two slave "<<std::endl;
                    if(!collision_avoidance(msg->lat,msg->lon)){
                        inf("this slave has to stop to avoid collision");
                        m_last_known_lat = msg->lat;
                        m_last_known_lon = msg->lon;
                        enableMovement(false);
                        return;
                    }

                }
                enableMovement(true);
                // compute the bearing with the announced data using previous data
                double announced_bearing;
                double announced_displace = 0;
                double bearing_difference;
                double last_known_speed;
                double bearing_master_slave;
                double displace_master_slave;
                //! these two variables are responsible for detecting the relative position between the master and slave to see how to handle the speed
                //!  of the master and slave in case the master and slave distance is more than the required offset we will adjust either the slave or the master speed
                if (!m_first_announce)
                {
                    WGS84::getNEBearingAndRange(m_last_known_lat, m_last_known_lon, msg->lat, msg->lon, &announced_bearing, &announced_displace);

                    // if the announcing system has not moved much, use the previously computed bearing
                    if (announced_displace < m_args.min_displace)
                        announced_bearing = m_last_known_bearing;

                    // check if this bearing should be given more emphasis than the one computed using remote state
                    if (!m_heading_timestamp.overflow())
                        announced_bearing = m_remote_heading;

                    // change the variable m_path according to the offsets in m_maneuver
                    computeNEDOffsets(msg->lat, msg->lon, 0.0, announced_bearing);

                    m_last_known_bearing = announced_bearing;

                    // Compute time delta.
                    double tstep = m_delta.getDelta();
                    checkSafety_3(m_path.end_lat,m_path.end_lon);
                    float error= (r2-m_args.desired_distance)/10;
                    
                    float speed_diff = m_rpm_pid.step(tstep, error);
                    m_path.speed=speed_diff;          
                    std::cout<<"the error"<<error<<std::endl;
                    std::cout<<"the new speed"<<speed_diff<<std::endl;

                    if(m_has_estimated_state){  //! this if condition will calculate the difference in the bearing between the master position and the slave
                        WGS84::getNEBearingAndRange(msg->lat, msg->lon, m_estate.lat, m_estate.lon, &bearing_master_slave, &displace_master_slave);
                        bearing_difference   = getDifference(m_last_known_bearing,bearing_master_slave);
                        float error2= bearing_difference - c_half_pi;
                    }

                }
                else // it is the first time announce is running
                {
                    // compute lat and lon of the desired path
                    m_first_announce=false;
                    computeNEDOffsets(msg->lat, msg->lon, 0.0, 0.0);
                    m_path.speed = m_maneuver.speed;
                }
                m_path.lradius = m_args.loiter_radius;
                m_path.flags = IMC::DesiredPath::FL_DIRECT;
                m_path.speed_units = m_maneuver.speed_units;
                dispatch(m_path);
                // update "last" variables
                m_last_known_lat = msg->lat;
                m_last_known_lon = msg->lon;
                //display_vector();
                trace("system being pursued has heading: %0.2f and was displaced %0.2f", m_last_known_bearing, announced_displace);
                trace("this is %0.4f seconds past the maneuver's initial time", Clock::get() - m_start_time);
                trace("announce data: lat %0.5f, lon %0.5f, %0.2f, %0.4f", msg->lat, msg->lon, msg->height, msg->getTimeStamp());
                double offx, offy;
                WGS84::displacement(msg->lat, msg->lon, 0.0, m_path.end_lat, m_path.end_lon, 0.0, &offx, &offy);
                trace("offset: x %0.2f, %0.2f, %0.2f", offx, offy, m_estate.z);
            }
            //! Function to check if the vehicle is getting near to the next waypoint
            void
            onPathControlState(const IMC::PathControlState* pcs)
            {
                if (pcs->flags & IMC::PathControlState::FL_NEAR)
                    enableMovement(false);
            }
            void
            display_vector(){

                //for (auto elem : display) {
                  //  std::cout<<elem<<",";

                //}

            }
            void
            onStateReport(void)
            {
                if (!m_maneuver.duration)
                    return;

                // if the present location is unsafe then disable movement
                if (!checkSafety(m_last_known_lat, m_last_known_lon))
                    enableMovement(false);

                if (m_last_update.overflow())
                    signalError(DTR("timeout to receive new remote info was exceeded."));

                double delta = Clock::get() - m_start_time - m_maneuver.duration;
                if (delta >= 0)
                    signalCompletion();
                else
                    signalProgress((uint16_t)(Math::round(delta)));
            }

            //! Function to compute new point to send to vehicle considering offsets
            void
            computeNEDOffsets(double lat, double lon, double depth, double psi)
            {
                double offx, offy;

                m_path.end_lat = lat;
                m_path.end_lon = lon;
                m_path.end_z = depth + m_maneuver.z;
                m_path.end_z_units = m_maneuver.z_units;

                // compute the offsets in the NED frame
                offx = std::cos(psi) * m_maneuver.x
                       + std::cos(Angles::normalizeRadian(psi - DUNE::Math::c_half_pi)) * m_maneuver.y;

                offy = std::sin(psi) * m_maneuver.x
                       + std::sin(Angles::normalizeRadian(psi - DUNE::Math::c_half_pi)) * m_maneuver.y;

                WGS84::displace(offx, offy, &m_path.end_lat, &m_path.end_lon);

            }

            //! this function is responsible to return the minimum difference between two bearing angles
            //! we will use this one to know the corresponding position between the slave and the master to decide if we need to increase the speed or decrease it
            double
            getDifference(double bearing1, double bearing2) {

                return std::min( std::fmod((bearing1-bearing2+M_PI) , M_PI),std::fmod((bearing1-bearing2+M_PI) , M_PI)) ;
            }

            //! Routine for checking the safety of the vehicle's position
            //! this routine return true if the present location is safe
            //! and returns false otherwise
            bool
            checkSafety(double lat, double lon)
            {
                if (m_has_estimated_state)
                {
                    double x, y, r;

                    WGS84::displacement(m_estate.lat, m_estate.lon, 0.0, lat, lon, 0.0, &x, &y);
                    r = Math::norm((x - m_estate.x), (y - m_estate.y));
                    if (r < m_args.safe_distance)
                    {
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
                else
                {
                    return true;
                }
            }

            bool
            checkSafety_3(double lat, double lon)
            {
                if (m_has_estimated_state)
                {
                    double x, y, r;

                    WGS84::displacement(m_estate.lat, m_estate.lon, 0.0, lat, lon, 0.0, &x, &y);
                    r2 = Math::norm((x - m_estate.x), (y - m_estate.y));
                    r = Math::norm((x - m_estate.x), (y - m_estate.y));
                    if (r < m_args.safe_distance)
                    {
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
                else
                {
                    return true;
                }
            }
            bool
            checkSafety2()
            {
                if (m_has_estimated_state)
                {
                    double x, y, r;

                    WGS84::displacement(v_last_known_lat, v_last_known_lon, 0.0, s_last_known_lat, s_last_known_lon, 0.0, &x, &y);

                    r = Math::norm(x, y);
                    std::cout<< r;
                    if (r < m_args.safe_distance)
                    {
                        return false;
                    }
                    else
                    {
                        return true;
                    }
                }
                else
                {
                    return true;
                }
            }
            bool
            collision_avoidance(double lat, double lon) {
                if (m_has_estimated_state) {
                    double x1, y1, x2, y2;


                    WGS84::getNEBearingAndRange(v_last_known_lat, v_last_known_lon, lat, lon, &x1, &y1);


                    WGS84::getNEBearingAndRange(s_last_known_lat, s_last_known_lon, lat, lon, &x2, &y2);

                    if (y2 > y1) {
                        return true;

                    } else {
                        return false;
                    }
                }
                return false;
            }
            //! Function for enabling and disabling the control loops
            void
            enableMovement(bool enable)
            {
                const uint32_t mask = IMC::CL_PATH;

                if (enable)
                    setControl(mask);
                else
                    setControl(0);
            }


        };
    }
}

DUNE_TASK