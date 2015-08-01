{
	'targets':
	[
		{
			'target_name': 'ROSLink',
			'sources': [
				'ROSLink.cpp',
				'NodeSesh.cpp',
				'NodeSession.cpp'
			],
			'include_dirs':
			[
				'<!@(pkg-config --cflags-only-I opencv)'
				'<!@(pkg-config --cflags-only-I roscpp_cool)'
				'<!@(pkg-config --cflags-only-I image_transport_cool)'
			],
			'libraries':
			[
				'<!@(pkg-config --libs opencv)',
				'<!@(pkg-config --libs roscpp_cool)',
				'<!@(pkg-config --libs image_transport_cool)'
			],
			'cflags': [ '-std=c++11', '-lstdc++', '-Wall'],
			'cflags!': [ '-fno-exceptions', '-fno-exceptions' ],
			'conditions': [
				[ 'OS=="linux"', {
					'libraries': [ '/usr/lib/x86_64-linux-gnu/libjpeg.so'],			
					'cflags_cc!': [ '-fno-rtti', '-fno-exceptions' ],
    				'cflags_cc+': [ '-frtti' ]
				}],
				[ 'OS=="mac"', {
					'cflags_cc!': [ '-fno-rtti', '-fno-exceptions' ],
					'cflags_cc+': [ '-frtti', '-fPIC' ],
					'cflags': [ '-stdlib=libc++' ],
					'xcode_settings': {
						'OTHER_CPLUSPLUSFLAGS' : ['-std=c++11','-stdlib=libc++'],
						'OTHER_LDFLAGS': ['-stdlib=libc++'],
			            		'GCC_ENABLE_CPP_RTTI': 'YES',
			           		 'GCC_ENABLE_CPP_EXCEPTIONS': 'YES',
						'MACOSX_DEPLOYMENT_TARGET': '10.7'
					},
					'include_dirs':
					[
						'<!@(pkg-config opencv --cflags-only-I)',
					],
					'libraries':
					[
						'<!@(pkg-config --libs opencv)',
					]
				}]
			]
		}
	]
}