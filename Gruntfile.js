var exec = require('child_process').exec;

var procsToKill = ['roscore',
                   'rosmaster',
                   'lsd_slam_core',
                   'viewer',
                   'node',
                   'iojs'];


function cleanup() {
    procsToKill.forEach(function(procToKill) {
        exec('pkill -9 -e -f ' + procToKill);
    });
}

function handleSignal(signal) {
    console.log('\nCaught ' + signal);
    cleanup();
    process.exit();
}

process.on('SIGINT', function() { handleSignal('SIGINT') });
process.on('SIGTSTP', function() { handleSignal('SIGTSTP') });
process.on('SIGQUIT', function() { handleSignal('SIGQUIT') });
process.on('SIGSEGV', function() { handleSignal('SIGSEGV') });
process.on('SIGTERM', function() { handleSignal('SIGTERM') });

module.exports = function(grunt) {

    grunt.initConfig({
        concurrent: {
            launch: [
                'shell:roscore',
                'shell:lsd_slam',
                'shell:lsd_slam_viewer',
                'shell:node'
            ]
        },
        shell: {  
            options: { stderr: true },
            bower: { command: 'bower install' },
            gyp: { command: 'export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$PWD/ros_link/config; cd ros_link; node-gyp rebuild; cd ..' },
            gyp_debug: { command: 'export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$PWD/ros_link/config; cd ros_link; node-gyp rebuild --debug; cd ..' },
            roscore: { command: 'roscore' },
            lsd_slam: { command: 'rosrun lsd_slam_core live_slam image:=/nodejs_link/image camera_info:=/nodejs_link/camera_info' },
            lsd_slam_viewer: { command: 'rosrun lsd_slam_viewer viewer' },
            node: { command: 'node js/main.js'}
        },
    });

    grunt.loadNpmTasks('grunt-concurrent');
    grunt.loadNpmTasks('grunt-shell');
    grunt.registerTask('build', 'Building project components', ['shell:bower', 'shell:gyp'] );
    grunt.registerTask('build_debug', 'Building project components (debug)', ['shell:bower', 'shell:gyp_debug'] );
    grunt.registerTask('cleanup', 'Kill existing processes', cleanup);
    grunt.registerTask('default', 'Starting DatacenterAR', ['cleanup', 'concurrent:launch']);

};
