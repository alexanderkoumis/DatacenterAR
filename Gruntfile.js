var exec = require('child_process').exec;

var procsToKill = ['roscore',
                   'rosmaster',
                   'lsd_slam_core',
                   'viewer',
                   'node'];


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
            install: { command: 'bower install; cd ros_link; node-gyp rebuild; cd ..' },
            roscore: { command: 'roscore' },
            lsd_slam: { command: 'rosrun lsd_slam_core live_slam image:=/nodejs_link/image camera_info:=/nodejs_link/camera_info' },
            lsd_slam_viewer: { command: 'rosrun lsd_slam_viewer viewer' },
            node: { command: 'node js/main.js'}
        },
    });

    grunt.loadNpmTasks('grunt-concurrent');
    grunt.loadNpmTasks('grunt-shell');

    grunt.registerTask('install', 'Performing installation procedures', 'shell:install');
    grunt.registerTask('cleanup', 'Kill existing processes', cleanup);
    grunt.registerTask('default', 'Starting DatacenterAR', ['cleanup', 'concurrent:launch']);

};
